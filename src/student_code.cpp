#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  {
    // TODO Part 1.
    std::vector<Vector2D> output;
    for (int i = 0; i < points.size() - 1; ++i) {
      output.push_back((1 - this->t) * points[i] + this->t * points[i+1]);

    }
    return output;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    std::vector<Vector3D> output;
    for (int i = 0; i < points.size() - 1; ++i) {
      output.push_back((1 - t) * points[i] + t * points[i+1]);

    }
    return output;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    std::vector<Vector3D> input = points;
    while (input.size() != 1) {
      input = BezierPatch::evaluateStep(input, t);
    }
    return input[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    std::vector<Vector3D> output;
    for (int i = 0; i < this->controlPoints.size();++i) {
      output.push_back(BezierPatch::evaluate1D(this->controlPoints[i], u));
    }

    return BezierPatch::evaluate1D(output, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.

    Vector3D N( 0., 0., 0. );

    HalfedgeCIter h = this->halfedge();
    do {
       FaceCIter face = h->face();
       N += face->normal();

       HalfedgeCIter h_twin = h->twin();
       VertexCIter v = h_twin->vertex();
       h = h_twin->next();
   } while(h != this->halfedge());

    return N.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    if (e0->isBoundary()) {
      return e0;
    }
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h3 = h0->twin();

    HalfedgeIter h1 = h0->next();
    HalfedgeIter h4 = h3->next();

    HalfedgeIter h2 = h1->next();
    HalfedgeIter h5 = h4->next();

    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h1->vertex();
    VertexIter v2 = h5->vertex();
    VertexIter v3 = h2->vertex();

    EdgeIter e1 = h5->edge();
    EdgeIter e2 = h4->edge();
    EdgeIter e3 = h2->edge();
    EdgeIter e4 = h1->edge();

    v0->halfedge() = h4;
    v1->halfedge() = h1;
    v2->halfedge() = h5;
    v3->halfedge() = h2;

    f0->halfedge() = h0;
    f1->halfedge() = h3;

    e1->halfedge() = h5;
    e2->halfedge() = h4;
    e3->halfedge() = h2;
    e4->halfedge() = h1;

    h0->setNeighbors(h2, h3, v2, e0, f0);
    h1->setNeighbors(h3, h1->twin(), v1, e4, f1);

    h2->setNeighbors(h4, h2->twin(), v3, e3, f0);
    h3->setNeighbors(h5, h0, v3, e0, f1);

    h4->setNeighbors(h0, h4->twin(), v0, e2, f0);
    h5->setNeighbors(h1, h5->twin(), v2, e1, f1);

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h3 = h0->twin();

    HalfedgeIter h1 = h0->next();
    HalfedgeIter h4 = h3->next();

    HalfedgeIter h2 = h1->next();
    HalfedgeIter h5 = h4->next();

    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h1->vertex();
    VertexIter v2 = h5->vertex();
    VertexIter v3 = h2->vertex();

    EdgeIter e1 = h5->edge();
    EdgeIter e2 = h4->edge();
    EdgeIter e3 = h2->edge();
    EdgeIter e4 = h1->edge();

    //new elements
    EdgeIter e5 = newEdge();
    EdgeIter e6 = newEdge();
    EdgeIter e7 = newEdge();

    FaceIter f2 = newFace();
    FaceIter f3 = newFace();

    VertexIter v4 = newVertex();

    HalfedgeIter h10 = newHalfedge();
    HalfedgeIter h11 = newHalfedge();
    HalfedgeIter h12 = newHalfedge();
    HalfedgeIter h13 = newHalfedge();
    HalfedgeIter h14 = newHalfedge();
    HalfedgeIter h15 = newHalfedge();

    //vertex
    v4->position = (v0->position + v1->position)/ 2.0;
    v4->halfedge() = h0;

    //set edge elements
    e5->halfedge() = h13;
    e6->halfedge() = h15;
    e7->halfedge() = h10;

    //set face elements;
    f0->halfedge() = h0;
    f1->halfedge() = h13;
    f2->halfedge() = h12;
    f3->halfedge() = h15;

    //set halfedge elements
    h0->setNeighbors(h1, h3, v4, e0, f0);

    h1->setNeighbors(h10, h1->twin(), v1, e4, f0);

    h2->setNeighbors(h12, h2->twin(), v3, e3, f2);

    h3->setNeighbors(h13, h0, v1, e0, f1);

    h4->setNeighbors(h14, h4->twin(), v0, e2, f3);

    h5->setNeighbors(h3, h5->twin(), v2, e1, f1);

    h10->setNeighbors(h0, h11, v3, e7, f0);

    h11->setNeighbors(h2, h10, v4, e7, f2);

    h12->setNeighbors(h11, h15, v0, e6, f2);

    h13->setNeighbors(h5, h14, v4, e5, f1);

    h14->setNeighbors(h15, h13, v2, e5, f3);

    h15->setNeighbors(h4, h12, v4, e6, f3);

    return v4;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.

    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.

    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)

    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.

  }
}
