#include "test.h"
#include "geometry/halfedge.h"

static void expect_split(Halfedge_Mesh &mesh, Halfedge_Mesh::EdgeRef edge, Halfedge_Mesh const &after) {
	if (auto ret = mesh.split_edge(edge)) {
		if (auto msg = mesh.validate()) {
			throw Test::error("Invalid mesh: " + msg.value().second);
		}
		// check mesh shape:
		if (auto difference = Test::differs(mesh, after, Test::CheckAllBits)) {
			throw Test::error("Resulting mesh did not match expected: " + *difference);
		}
	} else {
		throw Test::error("split_edge rejected operation!");
	}
}

/*
Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Split edge on Edge: 1-4

After mesh:
0--1\
|\ | \
| \2--3
|  | /
4--5/

*/
Test test_a2_l2_split_edge_simple("a2.l2.split_edge.simple", []() {
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
		                         Vec3(1.25f, 0.0f, 0.0f),  Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0,4,5,2}, {0,2,1}, {1,2,3}, {2,5,3}
	});

	expect_split(mesh, edge, after);
});

/*
Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Split edge on Edge: 0-1

After mesh:
0--1--2\
|  /  | \
| /   |  3
|/    | /
4-----5/
*/
Test test_a2_l2_split_edge_boundary("a2.l2.split_edge.boundary", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0,3,4,1}, {1,4,2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f),  Vec3(0.05f, 1.05f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            						Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), 							Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0,4,1}, {1,4,5,2}, {2,5,3}
	});

	expect_split(mesh, edge, after);
});

/*
Initial mesh:
0--1\
|\   \
| \---2
|    /
3--4/

Split on edge on 0-2

After mesh:
0--1\
|\ / \
| 5---2
|/   /
3--4/
*/
Test test_a2_l2_split_edge_angled("a2.l2.split_edge.angled", []()
                                  {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 1.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0,3,4,2}, {0,2,1}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 1.0f),
		Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f),
		Vec3(0.6f, 0.55f, 0.0f),
	}, {
		{0, 5, 1}, {1, 5, 2}, {0, 3, 5}, {3, 4, 2, 5}
	});

	expect_split(mesh, edge, after); });

/*
Initial mesh:
0--1\
|  / \
| |  2
|/   /
3--4/

Split edge on Edge: 1-3

After mesh:
0---1
|\ / \
| 5   2
|/ \ /
3--4/
*/
Test test_a2_l2_split_edge_simple_2("a2.l2.split_edge.simple_2", []()
                                    {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 1.0f),
		Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{1,3,4,2}, {0,3,1}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 1.0f),
		Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f),
		Vec3(-0.1f, 0.15f, 0.5f),
	}, {
		{0, 5, 1}, {1, 5, 4, 2}, {0, 3, 5}, {3,4,5}
	});

	expect_split(mesh, edge, after); });

/*
Initial mesh:
0--1\
 \   \
  \---2


Split edge on Edge: 0--1

After mesh:
0-3-1\
 \ \ _\
  \---2
*/
Test test_a2_l2_flip_edge_bound_triangle("a2.l2.flip_edge.bound_triangle", []()
                                         {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		Vec3(2.2f, 0.0f, 0.0f),
	}, {
		{2,1,0}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		Vec3(2.2f, 0.0f, 0.0f), Vec3(0.05f, 1.05f, 0.0f)
	}, {
		{2,3,0}, {2,1,3}
	});

	expect_split(mesh, edge, after); });