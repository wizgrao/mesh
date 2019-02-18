#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
  void BezierCurve::evaluateStep()
  {
      std::vector<Vector2D> points;
      auto lastLevel = evaluatedLevels[evaluatedLevels.size() - 1];
      for (int i = 0; i < lastLevel.size() - 1; i++) {
          points.push_back((1-t)*lastLevel[i] + t*lastLevel[i+1]);
      }
      evaluatedLevels.push_back(points);
  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    vector<Vector3D> points;
    for (vector<Vector3D> p : controlPoints) {
        points.push_back(evaluate1D(p, u));
    }
    return evaluate1D(points, v);
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const {
      for (int i = 0; i < 3; i ++) {
        std::vector<Vector3D> newvecs;
          for (int i = 0; i < points.size() - 1; i++) {
              newvecs.push_back((1-t)*points[i] + t*points[i+1]);
          }
          points = newvecs;
      }
      return points[0];
  }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
    HalfedgeCIter e = halfedge();
    Vector3D norm(0,0,0);
    do {
        Vector3D p1 = e->next()->vertex()->position - position;
        Vector3D p2 = e->next()->next()->vertex()->position - position;
        norm += cross(p1, p2);
        e = e->twin()->next();
    }while(e != halfedge());
    return norm.unit();


  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.
    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    return newVertex();
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.


    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.


    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.


    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)


    // TODO Now flip any new edge that connects an old and new vertex.


    // TODO Finally, copy the new vertex positions into final Vertex::position.

    return;
  }
}
