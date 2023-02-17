#include "test.h"
#include "geometry/halfedge.h"

static void expect_flip(Halfedge_Mesh &mesh, Halfedge_Mesh::EdgeRef edge, Halfedge_Mesh const &after) {
	if (auto ret = mesh.flip_edge(edge)) {
		if (auto msg = mesh.validate()) {
			throw Test::error("Invalid mesh: " + msg.value().second);
		}
		// check if returned edge is the same edge
		if (ret != edge) {
			throw Test::error("Did not return the same edge!");
		}
		// check mesh shape:
		if (auto difference = Test::differs(mesh, after, Test::CheckAllBits)) {
			throw Test::error("Resulting mesh did not match expected: " + *difference);
		}
	} else {
		throw Test::error("flip_edge rejected operation!");
	}
}

/*
Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Flip edge on Edge: 1-4

After mesh:
0--1\
|\   \
| \---2
|    /
3--4/
*/
Test test_a2_l1_flip_edge_simple("a2.l1.flip_edge.simple", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0,3,4,1}, {1,4,2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0,3,4,2}, {0,2,1}
	});

	expect_flip(mesh, edge, after);
});

/*
Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Flip edge on Edge: 3-4

After mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/
*/

Test test_a2_l1_flip_edge_boundary("a2.l1.flip_edge.boundary", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0,3,4,1}, {1,4,2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	if (mesh.flip_edge(edge)) {
		throw Test::error("flip_edge should not work at the boundary.");
	}
});

// From Piazza
/*
Initial mesh:
0--1\
|\   \
| \---2
|    /
3--4/

Flip edge on Edge: 0-2

After mesh:
0--1\
|  / \
| |  2
|/   /
3--4/
*/
Test test_a2_l1_flip_edge_simple_2("a2.l1.flip_edge.simple_2", []()
                                   {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0,3,4,2}, {0,2,1}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{1,3,4,2}, {0,3,1}
	});

	expect_flip(mesh, edge, after); });

/*
Initial mesh:
0--1\
|  / \
| |  2
|/   /
3--4/

Flip edge on Edge: 1-3

After mesh:
0--1\
|\   \
| |   2
| \  /
3--4/
*/
Test test_a2_l1_flip_edge_simple_3("a2.l1.flip_edge.simple_3", []()
                                   {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{1,3,4,2}, {0,3,1}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{1,0,4,2}, {3,4,0}
	});

	expect_flip(mesh, edge, after); });

/*
Initial mesh:
0--1\
|\   \
| |   2
| \  /
3--4/

Flip edge on Edge: 0-4

After mesh:
0--1\
|    \
| ___ 2
|/   /
3--4/
*/
Test test_a2_l1_flip_edge_simple_4("a2.l1.flip_edge.simple_4", []()
                                   {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{4,2,1,0}, {4,0,3}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.edges.begin();
	for (auto e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
		if (!e->on_boundary()) {
			edge = e;
		}
	}

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0,3,2,1}, {3,4,2}
	});

	expect_flip(mesh, edge, after); });

/*
Initial mesh:
0--1\
|    \
| ___ 2
|/   /
3--4/

Flip edge on Edge: 3-2

After mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/
*/
Test test_a2_l1_flip_edge_simple5("a2.l1.flip_edge.simple_5", []()
                                  {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{1,0,3,2}, {3,4,2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.edges.begin();
	for (auto e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
		if (!e->on_boundary()) {
			edge = e;
		}
	}

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0,3,4,1}, {4,2,1}
	});

	expect_flip(mesh, edge, after); });

/*
Initial mesh:
0---1
|\  |
| | |
|  \|
2---3

Flip edge on Edge: 0-3

After mesh:
0---1
|  /|
| | |
|/  |
2---3
*/
Test test_a2_l1_flip_edge_rectangle("a2.l1.flip_edge.rectangle", []()
                                    {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0,3,1}, {0,2,3}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.edges.begin();
	for (auto e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
		if (!e->on_boundary()) {
			edge = e;
		}
	}

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0,2,1}, {1,2,3}
	});

	expect_flip(mesh, edge, after); });

/*
Initial mesh:
  /0--1\
 /  \   \
5	 |   2
 \	  \ /
  \3--4/

Flip edge on Edge: 3-2

After mesh:
  /0--1\
 /      \
5--------2
 \	    /
  \3--4/
*/
Test test_a2_l1_flip_edge_hex("a2.l1.flip_edge.hex", []()
                              {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f), Vec3(-3.0f, 0.0f, 0.0f)
	}, {
		{1,0,4,2}, {4,0,5,3}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.edges.begin();
	for (auto e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
		if (!e->on_boundary()) {
			edge = e;
		}
	}

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f), Vec3(-3.0f, 0.0f, 0.0f)
	}, {
		{1,0,5,2}, {2,5,3,4}
	});

	expect_flip(mesh, edge, after); });

/*
Initial mesh:
  /0--1\
 /  \   \
5	 |   2
 \	  \ /
  \3--4/

Flip edge on boundary edge

After mesh:
  /0--1\
 /  \   \
5	 |   2
 \	  \ /
  \3--4/
*/
Test test_a2_l1_flip_boundary_edge("a2.l1.flip_edge.boundary_2", []()
                                   {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f), Vec3(-3.0f, 0.0f, 0.0f)
	}, {
		{1,0,4,2}, {4,0,5,3}
	});
	for (auto e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
		if (e->on_boundary() && mesh.flip_edge(e)) {
			throw Test::error("flip_edge flipped a boundary edge!");
		}
	} });