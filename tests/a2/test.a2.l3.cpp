#include "test.h"
#include "geometry/halfedge.h"

static void expect_collapse(Halfedge_Mesh &mesh, Halfedge_Mesh::EdgeRef edge, Halfedge_Mesh const &after) {
	if (auto ret = mesh.collapse_edge(edge)) {
		if (auto msg = mesh.validate()) {
			throw Test::error("Invalid mesh: " + msg.value().second);
		}
		// check mesh shape:
		if (auto difference = Test::differs(mesh, after, Test::CheckAllBits)) {
			throw Test::error("Resulting mesh did not match expected: " + *difference);
		}
	} else {
		throw Test::error("collapse_edge rejected operation!");
	}
}

/*
BASIC CASE

Initial mesh:
0--1\
|  | \
2--3--4
|  | /
5--6/

Collapse Edge on Edge: 2-3

After mesh:
0-----1\
 \   /  \
  \ /    \
   2------3
  / \    /
 /   \  /
4-----5/
*/
Test test_a2_l3_collapse_edge_basic_simple("a2.l3.collapse_edge.basic.simple", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		  Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		 Vec3(-1.2f, 0.0f, 0.0f),   	 Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-1.0f, 0.0f), 		Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 3, 1}, 
		{2, 5, 6, 3}, 
		{1, 3, 4}, 
		{3, 6, 4}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		  Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		 			Vec3(0.0f, 0.0f, 0.0f),  			Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-1.0f, 0.0f), 		Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 1}, 
		{2, 4, 5}, 
		{1, 2, 3}, 
		{2, 5, 3}
	});

	expect_collapse(mesh, edge, after);
});

/*
EDGE CASE

Initial mesh:
0--1\
|\ | \
| \2--3
|  | /
4--5/

Collapse Edge on Edge: 0-1

After mesh:
    0--\
   / \  \
  /   \  \
 /     1--2
/      | /
3------4/
*/
Test test_a2_l3_collapse_edge_edge_boundary("a2.l3.collapse_edge.edge.boundary", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                         Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-0.7f, 0.0f), Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 1}, 
		{0, 4, 5, 2}, 
		{1, 2, 3}, 
		{2, 5, 3}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		       Vec3(0.05f, 1.05f, 0.0f), 
		                         Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-0.7f, 0.0f), Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 1, 2}, 
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});

	expect_collapse(mesh, edge, after);
});

// My tests
/*
Initial mesh:
0--1
|\/|
3/\2


Collapse edge on Edge: 0-1

After mesh:
0--1
|\/|
3/\2

*/
Test test_a2_l3_collapse_edge_stacked_triangle("a2.l3.collapse_edge.stacked_triangle", []()
                                               {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f), Vec3(1.2f, 0.0f, 0.0f), Vec3(-1.2f, 0.0f, 0.0f),
	}, {
		{2,1,0}, {0,1,3}
	});
	if (auto msg = mesh.validate()) {
		throw Test::error("Invalid mesh: " + msg.value().second);
	}

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(0.05f, 1.05f, 0.0f), Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f), Vec3(-1.2f, 0.0f, 0.0f),
	}, {
		{2,1,0}, {0,1,3}
	});

	try {
		expect_collapse(mesh, edge, after);
	} catch (Test::error& e) {
		std::string error = e.what();
		if (error != "collapse_edge rejected operation!") {
			throw Test::error("collapse_edge did not reject operation!");
		}
	} });

/*
Initial mesh:
0--1
 \ |
  \2


Collapse edge on Edge: 0-1

After mesh:
0--1
 \ |
  \2

*/
Test test_a2_l3_collapse_edge_triangle("a2.l3.collapse_edge.triangle", []()
                                       {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f), Vec3(1.2f, 0.0f, 0.0f)
	}, {
		{2,1,0}
	});
	if (auto msg = mesh.validate()) {
		throw Test::error("Invalid mesh: " + msg.value().second);
	}

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(0.05f, 1.05f, 0.0f), Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
	}, {
		{2,1,0}
	});

	try {
		expect_collapse(mesh, edge, after);
	} catch (Test::error& e) {
		std::string error = e.what();
		if (error != "collapse_edge rejected operation!") {
			throw Test::error("collapse_edge did not reject operation!");
		}
	} });

/*
Initial mesh:
   0
  /|\
 / 1 \
/ / \ \
3-----2
(Tetrahedron)

Collapse edge on Edge: 0-3

After mesh:
   0
  /|\
 / 1 \
/ / \ \
3-----2

*/
Test test_a2_l3_collapse_edge_tetrahedron("a2.l3.collapse_edge.tetrahedron", []()
                                          {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f), Vec3(1.2f, 0.0f, 0.0f), Vec3(-1.2f, 0.0f, 0.0f),
	}, {
		{0,3,1}, {0,1,2}, {1,3,2}, {0,2,3}
	});
	if (auto msg = mesh.validate()) {
		throw Test::error("Invalid mesh: " + msg.value().second);
	}

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f), Vec3(1.2f, 0.0f, 0.0f), Vec3(-1.2f, 0.0f, 0.0f),
	}, {
		{0,3,1}, {0,1,2}, {1,3,2}, {0,2,3}
	});
  
	if (auto msg = after.validate()) {
		throw Test::error("Invalid mesh: " + msg.value().second);
	}

		try {
		expect_collapse(mesh, edge, after);
	} catch (Test::error& e) {
		std::string error = e.what();
		if (error != "collapse_edge rejected operation!") {
			throw Test::error("collapse_edge did not reject operation!");
		}
	} });

/*
Initial mesh:
0--1
 \ |
  \2
(stacked)

Collapse edge on Edge: 0-1

After mesh:
0--1
 \ |
  \2
(stacked)

*/
Test test_a2_l3_collapse_edge_degenerate_stacked_triangle("a2.l3.collapse_edge.degenerate_stacked_triangle", []()
                                                          {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(1.1f, 1.0f, 0.0f), Vec3(1.2f, 0.0f, 0.0f), Vec3(-1.1f, 0.55f, 0.0f),
	}, {
		{0,1,2}, {2,1,0}
	});
	if (auto msg = mesh.validate()) {
		throw Test::error("Invalid mesh: " + msg.value().second);
	}

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(1.1f, 1.0f, 0.0f), Vec3(1.2f, 0.0f, 0.0f), Vec3(-1.1f, 0.55f, 0.0f),
	}, {
		{0,1,2}, {2,1,0}
	});
	if (auto msg = after.validate()) {
		throw Test::error("Invalid mesh: " + msg.value().second);
	}

	try {
		expect_collapse(mesh, edge, after);
	} catch (Test::error& e) {
		std::string error = e.what();
		if (error != "collapse_edge rejected operation!") {
			throw Test::error("collapse_edge did not reject operation!");
		}
	} });

/*
Initial mesh:
0--1\
|  | \
2--3--4
|  | /
5--6/

Collapse edge on Edge: 3--4

After mesh:
0--1
|  |
2--3
|  |
4--5
*/

Test test_a2_l3_collapse_edge_two_triangles("a2.l3.collapse_edge.two_triangles", []()
                                            {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		Vec3(-1.2f, 0.0f, 0.0f),   	Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-1.0f, 0.0f), 	Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0,2,3,1}, {2,5,6,3}, {1,3,4}, {3,6,4}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->twin->next->edge;
	// std::cout << edge->halfedge->twin->vertex->position << "\n";
	// std::cout << edge->halfedge->vertex->position << "\n";

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		Vec3(-1.2f, 0.0f, 0.0f), 	Vec3(1.75f, 0.0f, 0.0f),
		Vec3(-1.4f,-1.0f, 0.0f), 	Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0,2,3,1}, {2,4,5,3}
	});

	expect_collapse(mesh, edge, after); });

/*
Initial mesh:
  /0--1\
 / |  | \
7--2--3--4
 \ |  | /
  \5--6/

Collapse edge on Edge: 2-3

After mesh:
  /0--1\
 / | /  \
2--3-----4
 \ | \  /
  \5--6/
*/

Test test_a2_l3_collapse_edge_hexagon("a2.l3.collapse_edge.hexagon", []()
                                      {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		Vec3(-1.2f, 0.0f, 0.0f),   	Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-1.0f, 0.0f), 	Vec3(1.5f, -1.0f, 0.0f), Vec3(-2.3f, 0.0f, 1.0f),
	}, {
		{0,2,3,1}, {2,5,6,3}, {1,3,4}, {3,6,4}, {7,2,0}, {7,5,2}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		Vec3(-2.3f, 0.0f, 1.0f), 	Vec3(0.0f, 0.0f, 0.0f), Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-1.0f, 0.0f), 	Vec3(1.5f, -1.0f, 0.0f),
	}, {
		{0,2,3}, {0,3,1}, {1,3,4}, {2,5,3}, {3,5,6}, {3,6,4}
	});

	expect_collapse(mesh, edge, after); });

/*
Initial mesh:
  /0--1\
 /      \
2        3
 \      /
  \4--5/

Collapse edge on Edge: 4-5

After mesh:
  /0--1\
 /      \
2        3
 \      /
  \4---/
*/

Test test_a2_l3_collapse_edge_empty_hexagon("a2.l3.collapse_edge.empty_hexagon", []()
                                            {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		Vec3(-2.3f, 0.0f, 1.0f),	Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-1.0f, 0.0f), 	Vec3(1.5f, -1.0f, 0.0f),
	}, {
		{5,4,2,0,1,3}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		Vec3(-2.3f, 0.0f, 1.0f),	Vec3(2.3f, 0.0f, 0.0f),
		Vec3(0.05f,-1.0f, 0.0f)
	}, {
		{4,2,0,1,3}
	});

	expect_collapse(mesh, edge, after); });

/*
Initial mesh:
/0--1\
/ |  | \
2  |  |  3
|\ |  | /|
| \|  |/ |
|  4--5  |
| /|  |\ |
|/ |  | \|
6  |  |  7
\ |  | /
\8--9/

Collapse edge on Edge: 4-5

After mesh:
/0--1\
/ |  | \
2  |  |  3
|\ | /  /|
| \|/  / |
|  4---  |
| /|\  \ |
|/ | \  \|
5  |  |  6
\ |  | /
\7--8/
*/
Test test_a2_l3_collapse_edge_octagon("a2.l3.collapse_edge.octagon", []()
                                      {
 Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
  Vec3(-1.0f, 1.0f, 0.0f),  Vec3(1.0f, 1.0f, 0.0f),
  Vec3(-2.0f, 0.5f, 0.0f),    Vec3(2.0f, 0.5f, 0.0f),
  Vec3(-0.5f, 0.0f, 0.0f),  Vec3(0.5f, 0.0f, 0.0f),
  Vec3(-2.0f, -0.5f, 0.0f),  Vec3(2.0f, -0.5f, 0.0f), 
  Vec3(-1.0f, -1.0f, 0.0f),  Vec3(1.0f, -1.0f, 0.0f),
 }, {
  {0,2,4}, {0,4,5,1}, {1,5,3}, {3,5,7}, {5,9,7}, {4,8,9,5}, {4,6,8}, {2,6,4}
 });

 Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->twin->next->edge;

 Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
  Vec3(-1.0f, 1.0f, 0.0f),  Vec3(1.0f, 1.0f, 0.0f),
  Vec3(-2.0f, 0.5f, 0.0f),    Vec3(2.0f, 0.5f, 0.0f),  
  Vec3(0.0f, 0.0f, 0.0f),
  Vec3(-2.0f, -0.5f, 0.0f),  Vec3(2.0f, -0.5f, 0.0f), 
  Vec3(-1.0f, -1.0f, 0.0f),  Vec3(1.0f, -1.0f, 0.0f),
 }, {
  {0,2,4}, {0,4,1}, {1,4,3}, {3,4,6}, {4,8,6}, {4,7,8}, {4,5,7}, {2,5,4}
 });

 expect_collapse(mesh, edge, after); });
