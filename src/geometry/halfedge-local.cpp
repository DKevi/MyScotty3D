
#include "halfedge.h"

#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <iostream>

/******************************************************************
*********************** Local Operations **************************
******************************************************************/

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it cannot perform an operation (i.e., because
    the resulting mesh does not have a valid representation).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
 * add_face: add a standalone face to the mesh
 *  sides: number of sides
 *  radius: distance from vertices to origin
 *
 * We provide this method as an example of how to make new halfedge mesh geometry.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::add_face(uint32_t sides, float radius)
{
  // faces with fewer than three sides are invalid, so abort the operation:
  if (sides < 3)
    return std::nullopt;

  std::vector<VertexRef> face_vertices;
  // In order to make the first edge point in the +x direction, first vertex should
  //  be at -90.0f - 0.5f * 360.0f / float(sides) degrees, so:
  float const start_angle = (-0.25f - 0.5f / float(sides)) * 2.0f * PI_F;
  for (uint32_t s = 0; s < sides; ++s)
  {
    float angle = float(s) / float(sides) * 2.0f * PI_F + start_angle;
    VertexRef v = emplace_vertex();
    v->position = radius * Vec3(std::cos(angle), std::sin(angle), 0.0f);
    face_vertices.emplace_back(v);
  }

  assert(face_vertices.size() == sides);

  // assemble the rest of the mesh parts:
  FaceRef face = emplace_face(false);    // the face to return
  FaceRef boundary = emplace_face(true); // the boundary loop around the face

  std::vector<HalfedgeRef> face_halfedges; // will use later to set ->next pointers

  for (uint32_t s = 0; s < sides; ++s)
  {
    // will create elements for edge from a->b:
    VertexRef a = face_vertices[s];
    VertexRef b = face_vertices[(s + 1) % sides];

    // h is the edge on face:
    HalfedgeRef h = emplace_halfedge();
    // t is the twin, lies on boundary:
    HalfedgeRef t = emplace_halfedge();
    // e is the edge corresponding to h,t:
    EdgeRef e = emplace_edge(false); // false: non-sharp

    // set element data to something reasonable:
    //(most ops will do this with interpolate_data(), but no data to interpolate here)
    h->corner_uv = a->position.xy() / (2.0f * radius) + 0.5f;
    h->corner_normal = Vec3(0.0f, 0.0f, 1.0f);
    t->corner_uv = b->position.xy() / (2.0f * radius) + 0.5f;
    t->corner_normal = Vec3(0.0f, 0.0f, -1.0f);

    // thing -> halfedge pointers:
    e->halfedge = h;
    a->halfedge = h;
    if (s == 0)
      face->halfedge = h;
    if (s + 1 == sides)
      boundary->halfedge = t;

    // halfedge -> thing pointers (except 'next' -- will set that later)
    h->twin = t;
    h->vertex = a;
    h->edge = e;
    h->face = face;

    t->twin = h;
    t->vertex = b;
    t->edge = e;
    t->face = boundary;

    face_halfedges.emplace_back(h);
  }

  assert(face_halfedges.size() == sides);

  for (uint32_t s = 0; s < sides; ++s)
  {
    face_halfedges[s]->next = face_halfedges[(s + 1) % sides];
    face_halfedges[(s + 1) % sides]->twin->next = face_halfedges[s]->twin;
  }

  return face;
}

/*
 * bisect_edge: split an edge without splitting the adjacent faces
 *  e: edge to split
 *
 * returns: added vertex
 *
 * We provide this as an example for how to implement local operations.
 * (and as a useful subroutine!)
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e)
{
  // Phase 0: draw a picture
  //
  // before:
  //    ----h--->
  // v1 ----e--- v2
  //   <----t---
  //
  // after:
  //    --h->    --h2->
  // v1 --e-- vm --e2-- v2
  //    <-t2-    <--t--
  //

  // Phase 1: collect existing elements
  HalfedgeRef h = e->halfedge;
  HalfedgeRef t = h->twin;
  VertexRef v1 = h->vertex;
  VertexRef v2 = t->vertex;

  // Phase 2: Allocate new elements, set data
  VertexRef vm = emplace_vertex();
  vm->position = (v1->position + v2->position) / 2.0f;
  interpolate_data({v1, v2}, vm); // set bone_weights

  EdgeRef e2 = emplace_edge();
  e2->sharp = e->sharp; // copy sharpness flag

  HalfedgeRef h2 = emplace_halfedge();
  interpolate_data({h, h->next}, h2); // set corner_uv, corner_normal

  HalfedgeRef t2 = emplace_halfedge();
  interpolate_data({t, t->next}, t2); // set corner_uv, corner_normal

  // The following elements aren't necessary for the bisect_edge, but they are here to demonstrate phase 4
  FaceRef f_not_used = emplace_face();
  HalfedgeRef h_not_used = emplace_halfedge();

  // Phase 3: Reassign connectivity (careful about ordering so you don't overwrite values you may need later!)

  vm->halfedge = h2;

  e2->halfedge = h2;

  assert(e->halfedge == h); // unchanged

  // n.b. h remains on the same face so even if h->face->halfedge == h, no fixup needed (t, similarly)

  h2->twin = t;
  h2->next = h->next;
  h2->vertex = vm;
  h2->edge = e2;
  h2->face = h->face;

  t2->twin = h;
  t2->next = t->next;
  t2->vertex = vm;
  t2->edge = e;
  t2->face = t->face;

  h->twin = t2;
  h->next = h2;
  assert(h->vertex == v1); // unchanged
  assert(h->edge == e);    // unchanged
  // h->face unchanged

  t->twin = h2;
  t->next = t2;
  assert(t->vertex == v2); // unchanged
  t->edge = e2;
  // t->face unchanged

  // Phase 4: Delete unused elements
  erase_face(f_not_used);
  erase_halfedge(h_not_used);

  // Phase 5: Return the correct iterator
  return vm;
}

/*
 * split_edge: split an edge and adjacent (non-boundary) faces
 *  e: edge to split
 *
 * returns: added vertex. vertex->halfedge should lie along e
 *
 * Note that when splitting the adjacent faces, the new edge
 * should connect to the vertex ccw from the ccw-most end of e
 * within the face.
 *
 * Do not split adjacent boundary faces.
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(EdgeRef e)
{
  // A2L2 (REQUIRED): split_edge

  // Phase 1: collect existing elements
  HalfedgeRef h = e->halfedge;
  HalfedgeRef t = h->twin;
  VertexRef v1 = h->vertex;
  VertexRef v2 = t->vertex;
  HalfedgeRef hNext = h->next;
  HalfedgeRef tNext = t->next;
  HalfedgeRef hNextNext = hNext->next;
  HalfedgeRef tNextNext = tNext->next;
  // Phase 2: Allocate new elements, set data
  VertexRef vm = emplace_vertex();
  vm->position = (v1->position + v2->position) / 2.0f;
  interpolate_data({v1, v2}, vm); // set bone_weights

  EdgeRef e2 = emplace_edge();
  e2->sharp = e->sharp; // copy sharpness flag

  HalfedgeRef h2 = emplace_halfedge();
  interpolate_data({h, h->next}, h2); // set corner_uv, corner_normal

  HalfedgeRef t2 = emplace_halfedge();
  interpolate_data({t, t->next}, t2); // set corner_uv, corner_normal

  vm->halfedge = h2;

  e2->halfedge = h2;

  assert(e->halfedge == h); // unchanged

  h2->twin = t;
  h2->next = h->next;
  h2->vertex = vm;
  h2->edge = e2;
  h2->face = h->face;

  t2->twin = h;
  t2->next = t->next;
  t2->vertex = vm;
  t2->edge = e;
  t2->face = t->face;

  h->twin = t2;
  h->next = h2;
  assert(h->vertex == v1); // unchanged
  assert(h->edge == e);    // unchanged
  // h->face unchanged

  t->twin = h2;
  t->next = t2;
  assert(t->vertex == v2); // unchanged
  t->edge = e2;
  // t->face unchanged

  FaceRef hFace = h->face;
  if (!hFace->boundary)
  {
    EdgeRef hSideNewEdge = emplace_edge();
    hSideNewEdge->sharp = e->sharp; // copy sharpness flag
    HalfedgeRef hSideNewEdgeHalf = emplace_halfedge();
    interpolate_data({hNext, h2}, hSideNewEdgeHalf); // set corner_uv, corner_normal
    HalfedgeRef hSideNewEdgeTwin = emplace_halfedge();
    interpolate_data({h, hNextNext}, hSideNewEdgeTwin); // set corner_uv, corner_normal
    FaceRef hSideNewFace = emplace_face();
    hSideNewFace->halfedge = hSideNewEdgeHalf;
    hSideNewEdge->halfedge = hSideNewEdgeHalf;

    hSideNewEdgeHalf->twin = hSideNewEdgeTwin;
    hSideNewEdgeHalf->next = h2;
    hSideNewEdgeHalf->vertex = hNextNext->vertex;
    hSideNewEdgeHalf->edge = hSideNewEdge;
    hSideNewEdgeHalf->face = hSideNewFace;

    hSideNewEdgeTwin->twin = hSideNewEdgeHalf;
    hSideNewEdgeTwin->next = hNextNext;
    hSideNewEdgeTwin->vertex = vm;
    hSideNewEdgeTwin->edge = hSideNewEdge;
    hSideNewEdgeTwin->face = hFace;

    h->next = hSideNewEdgeTwin;
    hNext->next = hSideNewEdgeHalf;
    h2->face = hSideNewFace;
    hNext->face = hSideNewFace;
    hFace->halfedge = h;
  }

  FaceRef tFace = t->face;
  if (!tFace->boundary)
  {
    EdgeRef tSideNewEdge = emplace_edge();
    tSideNewEdge->sharp = e->sharp; // copy sharpness flag
    HalfedgeRef tSideNewEdgeHalf = emplace_halfedge();
    interpolate_data({tNext, t2}, tSideNewEdgeHalf); // set corner_uv, corner_normal
    HalfedgeRef tSideNewEdgeTwin = emplace_halfedge();
    interpolate_data({t, tNextNext}, tSideNewEdgeTwin); // set corner_uv, corner_normal
    FaceRef tSideNewFace = emplace_face();
    tSideNewFace->halfedge = tSideNewEdgeHalf;
    tSideNewEdge->halfedge = tSideNewEdgeHalf;

    tSideNewEdgeHalf->twin = tSideNewEdgeTwin;
    tSideNewEdgeHalf->next = t2;
    tSideNewEdgeHalf->vertex = tNextNext->vertex;
    tSideNewEdgeHalf->edge = tSideNewEdge;
    tSideNewEdgeHalf->face = tSideNewFace;

    tSideNewEdgeTwin->twin = tSideNewEdgeHalf;
    tSideNewEdgeTwin->next = tNextNext;
    tSideNewEdgeTwin->vertex = vm;
    tSideNewEdgeTwin->edge = tSideNewEdge;
    tSideNewEdgeTwin->face = tFace;

    t->next = tSideNewEdgeTwin;
    tNext->next = tSideNewEdgeHalf;
    t2->face = tSideNewFace;
    tNext->face = tSideNewFace;
    tFace->halfedge = t;
  }
  // Phase 5: Return the correct iterator
  return vm;
}

/*
 * inset_vertex: divide a face into triangles by placing a vertex at f->center()
 *  f: the face to add the vertex to
 *
 * returns:
 *  std::nullopt if insetting a vertex would make mesh invalid
 *  the inset vertex otherwise
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::inset_vertex(FaceRef f)
{
  // A2Lx4 (OPTIONAL): inset vertex

  (void)f;
  return std::nullopt;
}

/* [BEVEL NOTE] Note on the beveling process:

  Each of the bevel_vertex, bevel_edge, and extrude_face functions do not represent
  a full bevel/extrude operation. Instead, they should update the _connectivity_ of
  the mesh, _not_ the positions of newly created vertices. In fact, you should set
  the positions of new vertices to be exactly the same as wherever they "started from."

  When you click on a mesh element while in bevel mode, one of those three functions
  is called. But, because you may then adjust the distance/offset of the newly
  beveled face, we need another method of updating the positions of the new vertices.

  This is where bevel_positions and extrude_positions come in: these functions are
  called repeatedly as you move your mouse, the position of which determines the
  amount / shrink parameters. These functions are also passed an array of the original
  vertex positions, stored just after the bevel/extrude call, in order starting at
  face->halfedge->vertex, and the original element normal, computed just *before* the
  bevel/extrude call.

  Finally, note that the amount, extrude, and/or shrink parameters are not relative
  values -- you should compute a particular new position from them, not a delta to
  apply.
*/

/*
 * bevel_vertex: creates a face in place of a vertex
 *  v: the vertex to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(VertexRef v)
{
  // A2Lx5 (OPTIONAL): Bevel Vertex
  //  Reminder: This function does not update the vertex positions.
  //  Remember to also fill in bevel_vertex_helper (A2Lx5h)

  (void)v;
  return std::nullopt;
}

/*
 * bevel_edge: creates a face in place of an edge
 *  e: the edge to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(EdgeRef e)
{
  // A2Lx6 (OPTIONAL): Bevel Edge
  //  Reminder: This function does not update the vertex positions.
  //  remember to also fill in bevel_edge_helper (A2Lx6h)

  (void)e;
  return std::nullopt;
}

/*
 * extrude_face: creates a face inset into a face
 *  f: the face to inset
 *
 * returns: reference to the inner face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::extrude_face(FaceRef f)
{
  // A2L4: Extrude Face
  //  Reminder: This function does not update the vertex positions.
  //  Remember to also fill in extrude_helper (A2L4h)
  std::unordered_map<VertexRef, VertexRef> vertexMap;
  HalfedgeRef og = f->halfedge;
  HalfedgeRef h = og;
  do
  {
    VertexRef v = h->vertex;

    VertexRef vExtrude = emplace_vertex();
    vExtrude->position = v->position;
    vExtrude->bone_weights = v->bone_weights;
    
    vertexMap[v] = vExtrude;

    h = h->next;
  } while (h != og);

  h = og;
  do
  {
    EdgeRef e = h->edge;
    HalfedgeRef t = h->twin;
    VertexRef v = h->vertex;
    VertexRef vNext = h->next->vertex;
    VertexRef vExtrude = vertexMap[v];
    VertexRef vNextExtrude = vertexMap[vNext];

    EdgeRef eExtrude = emplace_edge();
    eExtrude->sharp = e->sharp;

    HalfedgeRef h2 = emplace_halfedge();
    h2->corner_normal = h->corner_normal;
    h2->corner_uv = h->corner_uv;
    HalfedgeRef t2 = emplace_halfedge();
    t2->corner_normal = t->corner_normal;
    t2->corner_uv = t->corner_uv;

    eExtrude->halfedge = h2;
    vExtrude->halfedge = h2;

    h2->twin = t2;
    h2->vertex = vExtrude;
    h2->edge = eExtrude;

    t2->twin = h2;
    t2->vertex = vNextExtrude;
    t2->edge = eExtrude;

    h = h->next;
  } while (h != og);

  h = og;
  do
  {
    EdgeRef e = h->edge;
    VertexRef v = h->vertex;
    VertexRef vNext = h->next->vertex;
    VertexRef vExtrude = vertexMap[v];
    VertexRef vNextExtrude = vertexMap[vNext];
    HalfedgeRef h2 = vExtrude->halfedge;
    HalfedgeRef t2 = h2->twin;
    HalfedgeRef h2Next = vNextExtrude->halfedge;

    EdgeRef eInter = emplace_edge();
    eInter->sharp = e->sharp;

    HalfedgeRef hInter = emplace_halfedge();
    hInter->corner_normal = h->corner_normal;
    hInter->corner_uv = h->corner_uv;
    HalfedgeRef tInter = emplace_halfedge();
    tInter->corner_normal = h->corner_normal;
    tInter->corner_uv = h->corner_uv;

    eInter->halfedge = hInter;

    hInter->twin = tInter;
    hInter->vertex = v;
    hInter->edge = eInter;

    tInter->twin = hInter;
    tInter->next = h;
    tInter->vertex = vExtrude;
    tInter->edge = eInter;

    h2->face = f;
    t2->next = tInter;
    h2->next = h2Next;
    f->halfedge = h2;
        
    h = h->next;
  } while (h != og);

  h = og;
  do
  {
    VertexRef v = h->vertex;
    VertexRef vNext = h->next->vertex;
    VertexRef vExtrude = vertexMap[v];
    VertexRef vNextExtrude = vertexMap[vNext];
    HalfedgeRef hNext = h->next;
    HalfedgeRef h2 = vExtrude->halfedge;
    HalfedgeRef t2 = h2->twin;
    HalfedgeRef h2Next = vNextExtrude->halfedge;
    HalfedgeRef t2Next = h2Next->twin;
    HalfedgeRef tInner = t2Next->next;
    HalfedgeRef hInner = tInner->twin;

    FaceRef fSide = emplace_face(false);
    fSide->halfedge = h;
    h->next = hInner;
    hInner->next = t2;    

    h->face = fSide;
    hInner->face = fSide;
    t2->face = fSide;
    t2->next->face = fSide;

    h = hNext;
  } while (h != og);

  return f;
}

/*
 * flip_edge: rotate non-boundary edge ccw inside its containing faces
 *  e: edge to flip
 *
 * if e is a boundary edge, does nothing and returns std::nullopt
 * if flipping e would create an invalid mesh, does nothing and returns std::nullopt
 *
 * otherwise returns the edge, post-rotation
 *
 * does not create or destroy mesh elements.
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(EdgeRef e)
{
  // A2L1: Flip Edge
  //  Get references
  HalfedgeRef h = e->halfedge;
  HalfedgeRef t = h->twin;
  if (e->on_boundary())
    return std::nullopt;
  HalfedgeRef hNext = h->next;
  HalfedgeRef tNext = t->next;
  HalfedgeRef hNextNext = h->next->next;
  HalfedgeRef tNextNext = t->next->next;
  // Update hNext and tNext face
  hNext->face->halfedge = h;
  tNext->face->halfedge = t;
  hNext->face = t->face;
  tNext->face = h->face;
  // Find hPrev and tPrev
  HalfedgeRef hPrev = hNext;
  while (hPrev->next != h)
  {
    hPrev = hPrev->next;
  }
  HalfedgeRef tPrev = tNext;
  while (tPrev->next != t)
  {
    tPrev = tPrev->next;
  }
  if ((hNext->twin == tPrev) || (tNext->twin == hPrev))
    return std::nullopt;
  // Update hPrev hNext tPrev and tNext
  hPrev->next = tNext;
  hNext->next = t;
  tPrev->next = hNext;
  tNext->next = h;
  // Update h and t
  h->vertex->halfedge = tNext;
  t->vertex->halfedge = hNext;
  h->vertex = tNextNext->vertex;
  t->vertex = hNextNext->vertex;
  h->next = hNextNext;
  t->next = tNextNext;

  return h->edge;
}

/*
 * make_boundary: add non-boundary face to boundary
 *  face: the face to make part of the boundary
 *
 * if face ends up adjacent to other boundary faces, merge them into face
 *
 * if resulting mesh would be invalid, does nothing and returns std::nullopt
 * otherwise returns face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::make_boundary(FaceRef face)
{
  // A2Lx7: (OPTIONAL) make_boundary

  return std::nullopt; // TODO: actually write this code!
}

/*
 * dissolve_vertex: merge non-boundary faces adjacent to vertex, removing vertex
 *  v: vertex to merge around
 *
 * if merging would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_vertex(VertexRef v)
{
  // A2Lx1 (OPTIONAL): Dissolve Vertex

  return std::nullopt;
}

/*
 * dissolve_edge: merge the two faces on either side of an edge
 *  e: the edge to dissolve
 *
 * merging a boundary and non-boundary face produces a boundary face.
 *
 * if the result of the merge would be an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_edge(EdgeRef e)
{
  // A2Lx2 (OPTIONAL): dissolve_edge

  // Reminder: use interpolate_data() to merge corner_uv / corner_normal data

  return std::nullopt;
}

/* collapse_edge: collapse edge to a vertex at its middle
 *  e: the edge to collapse
 *
 * if collapsing the edge would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(EdgeRef e)
{
  // A2L3: Collapse Edge
  HalfedgeRef h = e->halfedge;
  HalfedgeRef t = h->twin;
  VertexRef v1 = h->vertex;
  VertexRef v2 = t->vertex;

  if (h->face->halfedge == h)
  {
    h->face->halfedge = h->next;
  }
  if (t->face->halfedge == t)
  {
    t->face->halfedge = t->next;
  }

  std::unordered_set<VertexRef> v1Vertices, v2Vertices, commonVertices;
  std::unordered_map<VertexRef, EdgeRef> removeMap;

  HalfedgeRef v1HalfEdge = t->next;
  while (v1HalfEdge != h)
  {
    HalfedgeRef v1HalfEdgeTwin = v1HalfEdge->twin;
    VertexRef v1V = v1HalfEdgeTwin->vertex;
    if (v1V->id != v2->id) {
      v1Vertices.emplace(v1V);
    }
    v1HalfEdge = v1HalfEdgeTwin->next;
  }

  HalfedgeRef v2HalfEdge = h->next;
  while (v2HalfEdge != t)
  {
    HalfedgeRef v2HalfEdgeTwin = v2HalfEdge->twin;
    VertexRef v2V = v2HalfEdgeTwin->vertex;
    if (v2V->id != v1->id) {
      v2Vertices.emplace(v2V);
      if (v1Vertices.count(v2V)) {
        commonVertices.emplace(v2V);
      }
    }
    v2HalfEdge = v2HalfEdgeTwin->next;
  }
  
  v2HalfEdge = h->next;
  while (v2HalfEdge != t)
  {
    HalfedgeRef v2HalfEdgeTwin = v2HalfEdge->twin;
    VertexRef v2V = v2HalfEdgeTwin->vertex;
    if (v2V->id != v1->id)
    {
      if (v1Vertices.count(v2V))
      {
        bool isTriangle = true;
        HalfedgeRef commonVertexHalfedge = v2V->halfedge;
        for (uint32_t i = 0; i < v2V->degree(); i++)
        {
          HalfedgeRef commonVertexHalfedgeTwin = commonVertexHalfedge->twin;
          if ((commonVertexHalfedgeTwin->vertex != v1) && (commonVertexHalfedgeTwin->vertex != v2)) {
            if (commonVertices.count(commonVertexHalfedgeTwin->vertex) == 0)
              isTriangle = false;
          }
          commonVertexHalfedge = commonVertexHalfedgeTwin->next;
        }
        if (isTriangle)
          return std::nullopt;
      }
    }
    v2HalfEdge = v2HalfEdgeTwin->next;
  }

  VertexRef vm = bisect_edge(e).value();
  vm->halfedge = t->next->next;
  v1HalfEdge = t->next->next;
  while (v1HalfEdge != h)
  {
    HalfedgeRef v1HalfEdgeTwin = v1HalfEdge->twin;
    VertexRef v1V = v1HalfEdgeTwin->vertex;
    if (v1V->id != vm->id) {
      v1HalfEdge->vertex = vm;
      interpolate_data({t->next, v1HalfEdge}, v1HalfEdge);
      interpolate_data({v1HalfEdgeTwin, h}, v1HalfEdgeTwin);
      if (v2Vertices.count(v1V)) {
        removeMap[v1V] = v1HalfEdge->edge;
      }
      if (v1HalfEdgeTwin->next == h) {
        v1HalfEdgeTwin->next = h->next->next;
        break;
      }
    }
    v1HalfEdge = v1HalfEdgeTwin->next;
  }
  v2HalfEdge = h->next->next;
  while (v2HalfEdge != t)
  {
    HalfedgeRef v2HalfEdgeTwin = v2HalfEdge->twin;
    VertexRef v2V = v2HalfEdgeTwin->vertex;
    HalfedgeRef v2HalfEdgeNext = v2HalfEdgeTwin->next;
    if (v2V->id != vm->id) {
      v2HalfEdge->vertex = vm;
      interpolate_data({h->next, v2HalfEdge}, v2HalfEdge);
      interpolate_data({v2HalfEdgeTwin, t}, v2HalfEdgeTwin);
      if (v1Vertices.count(v2V))
      {
        HalfedgeRef eraseInsideHalfEdge = removeMap[v2V]->halfedge;
        if ((eraseInsideHalfEdge->face != v2HalfEdge->face) && (eraseInsideHalfEdge->face!= v2HalfEdgeTwin->face))
          eraseInsideHalfEdge = eraseInsideHalfEdge->twin;

        HalfedgeRef eraseOutsideHalfedge = eraseInsideHalfEdge->twin;
        EdgeRef eraseEdge = eraseInsideHalfEdge->edge;
        FaceRef eraseFace = eraseInsideHalfEdge->face;
        HalfedgeRef eraseOutsideHalfedgePrev = eraseOutsideHalfedge->next;
        while (eraseOutsideHalfedgePrev->next != eraseOutsideHalfedge)
          eraseOutsideHalfedgePrev = eraseOutsideHalfedgePrev->next;

        HalfedgeRef keepInsideHalfedge = v2HalfEdge;
        if (keepInsideHalfedge->face->id != eraseFace->id)
          keepInsideHalfedge = keepInsideHalfedge->twin;

        keepInsideHalfedge->next = eraseOutsideHalfedge->next;
        keepInsideHalfedge->face = eraseOutsideHalfedge->face;
        eraseOutsideHalfedgePrev->next = keepInsideHalfedge;
        interpolate_data({keepInsideHalfedge, eraseOutsideHalfedge}, keepInsideHalfedge);
        interpolate_data({keepInsideHalfedge->twin, eraseInsideHalfEdge}, keepInsideHalfedge->twin);
        
        vm->halfedge = keepInsideHalfedge;
        if (vm->halfedge->vertex != vm)
          vm->halfedge = vm->halfedge->twin;

        v2V->halfedge = keepInsideHalfedge;
        if (v2V->halfedge->vertex != v2V)
          v2V->halfedge = v2V->halfedge->twin;

        eraseOutsideHalfedge->face->halfedge = keepInsideHalfedge;
        erase_face(eraseFace);
        erase_edge(eraseEdge);
        erase_halfedge(eraseInsideHalfEdge);
        erase_halfedge(eraseOutsideHalfedge);
      }
      if (v2HalfEdgeTwin->next == t) {
        v2HalfEdgeTwin->next = t->next->next;
        break;
      }
    }
    v2HalfEdge = v2HalfEdgeNext;
  }

  erase_vertex(v1);
  erase_vertex(v2);
  erase_edge(h->edge);
  erase_edge(h->next->edge);
  erase_halfedge(h->next);
  erase_halfedge(h);
  erase_halfedge(t->next);
  erase_halfedge(t);
  return vm;
}

/*
 * collapse_face: collapse a face to a single vertex at its center
 *  f: the face to collapse
 *
 * if collapsing the face would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(FaceRef f)
{
  // A2Lx3 (OPTIONAL): Collapse Face

  // Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
  //  (also works for bone_weights data on vertices!)

  return std::nullopt;
}

/*
 * weld_edges: glue two boundary edges together to make one non-boundary edge
 *  e, e2: the edges to weld
 *
 * if welding the edges would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns e, updated to represent the newly-welded edge
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::weld_edges(EdgeRef e, EdgeRef e2)
{
  // A2Lx8: Weld Edges

  // Reminder: use interpolate_data() to merge bone_weights data on vertices!

  return std::nullopt;
}

/*
 * bevel_positions: compute new positions for the vertices of a beveled vertex/edge
 *  face: the face that was created by the bevel operation
 *  start_positions: the starting positions of the vertices
 *     start_positions[i] is the starting position of face->halfedge(->next)^i
 *  direction: direction to bevel in (unit vector)
 *  distance: how far to bevel
 *
 * push each vertex from its starting position along its outgoing edge until it has
 *  moved distance `distance` in direction `direction`. If it runs out of edge to
 *  move along, you may choose to extrapolate, clamp the distance, or do something
 *  else reasonable.
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after bevel_vertex or bevel_edge.
 * (So you can assume the local topology is set up however your bevel_* functions do it.)
 *
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::bevel_positions(FaceRef face, std::vector<Vec3> const &start_positions, Vec3 direction, float distance)
{
  // A2Lx5h / A2Lx6h (OPTIONAL): Bevel Positions Helper

  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the start_positions array) to compute an new vertex position.
}

/*
 * extrude_positions: compute new positions for the vertices of an extruded face
 *  face: the face that was created by the extrude operation
 *  move: how much to translate the face
 *  shrink: amount to linearly interpolate vertices in the face toward the face's centroid
 *    shrink of zero leaves the face where it is
 *    positive shrink makes the face smaller (at shrink of 1, face is a point)
 *    negative shrink makes the face larger
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after extrude_face.
 * (So you can assume the local topology is set up however your extrude_face function does it.)
 *
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::extrude_positions(FaceRef face, Vec3 move, float shrink)
{
  // A2L4h: Extrude Positions Helper

  // General strategy:
  //  use mesh navigation to get starting positions from the surrounding faces,
  //  compute the centroid from these positions + use to shrink,
  //  offset by move
  HalfedgeRef h = face->halfedge;
  do {
    VertexRef v = h->vertex;
    VertexRef vOuter = h->twin->next->next->vertex;
    v->position = vOuter->position * (1 - shrink);
    v->position += move;
    h = h->next;
  } while (h != face->halfedge);
}
