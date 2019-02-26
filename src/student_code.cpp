#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL {
    void BezierCurve::evaluateStep() {
        std::vector<Vector2D> points;
        auto lastLevel = evaluatedLevels[evaluatedLevels.size() - 1];
        for (int i = 0; i < lastLevel.size() - 1; i++) {
            points.push_back((1 - t) * lastLevel[i] + t * lastLevel[i + 1]);
        }
        evaluatedLevels.push_back(points);
    }


    Vector3D BezierPatch::evaluate(double u, double v) const {
        vector<Vector3D> points;
        for (vector<Vector3D> p : controlPoints) {
            points.push_back(evaluate1D(p, u));
        }
        return evaluate1D(points, v);
    }

    Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const {
        for (int i = 0; i < 3; i++) {
            std::vector<Vector3D> newvecs;
            for (int i = 0; i < points.size() - 1; i++) {
                newvecs.push_back((1 - t) * points[i] + t * points[i + 1]);
            }
            points = newvecs;
        }
        return points[0];
    }


    Vector3D Vertex::normal(void) const {
        HalfedgeCIter e = halfedge();
        Vector3D norm(0, 0, 0);
        do {
            Vector3D p1 = e->next()->vertex()->position - position;
            Vector3D p2 = e->next()->next()->vertex()->position - position;
            norm += cross(p1, p2);
            e = e->twin()->next();
        } while (e != halfedge());
        return norm.unit();


    }

    EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
        if (e0->isBoundary())
            return e0;

        auto h0 = e0->halfedge();
        auto h1 = h0->next();
        auto h2 = h1->next();

        auto h3 = h0->twin();
        auto h4 = h3->next();
        auto h5 = h4->next();

        auto h6 = h1->twin();
        auto h7 = h2->twin();
        auto h8 = h4->twin();
        auto h9 = h5->twin();

        auto v0 = h0->vertex();
        auto v1 = h2->vertex();
        auto v2 = h1->vertex();
        auto v3 = h5->vertex();

        auto f0 = h0->face();
        auto f1 = h3->face();

        auto e1 = h1->edge();
        auto e2 = h2->edge();
        auto e3 = h4->edge();
        auto e4 = h5->edge();

        h0->setNeighbors(h2, h3, v3, e0, f0);
        h1->setNeighbors(h3, h6, v2, e1, f1);
        h2->setNeighbors(h4, h7, v1, e2, f0);

        h3->setNeighbors(h5, h0, v1, e0, f1);
        h4->setNeighbors(h0, h8, v0, e3, f0);
        h5->setNeighbors(h1, h9, v3, e4, f1);

        f0->halfedge() = h0;
        f1->halfedge() = h3;

        v0->halfedge() = h4;
        v1->halfedge() = h3;
        v2->halfedge() = h1;
        v3->halfedge() = h5;

        return e0;
    }

    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
        if (e0->isBoundary())
            return newVertex();

        auto h0 = e0->halfedge();
        auto h1 = h0->next();
        auto h2 = h1->next();

        auto h3 = h0->twin();
        auto h4 = h3->next();
        auto h5 = h4->next();

        auto h6 = h1->twin();
        auto h7 = h2->twin();
        auto h8 = h4->twin();
        auto h9 = h5->twin();

        auto v0 = h0->vertex();
        auto v1 = h2->vertex();
        auto v2 = h1->vertex();
        auto v3 = h5->vertex();

        auto f0 = h0->face();
        auto f1 = h3->face();

        auto e1 = h1->edge();
        auto e2 = h2->edge();
        auto e3 = h4->edge();
        auto e4 = h5->edge();

        auto v4 = newVertex();

        auto h0p = newHalfedge();
        auto h3p = newHalfedge();
        auto h10 = newHalfedge();
        auto h11 = newHalfedge();
        auto h12 = newHalfedge();
        auto h13 = newHalfedge();

        auto e0p = newEdge();
        auto e5 = newEdge();
        auto e6 = newEdge();

        auto f0p = newFace();
        auto f1p = newFace();

        f0->halfedge() = h0;
        f1->halfedge() = h3;
        f0p->halfedge() = h0p;
        f1p->halfedge() = h3p;

        e0p->halfedge() = h0p;
        e5->halfedge() = h12;
        e6->halfedge() = h13;

        v0->halfedge() = h0;
        v1->halfedge() = h10;
        v2->halfedge() = h3p;
        v3->halfedge() = h13;
        v4->halfedge() = h3;

        h0->setNeighbors(h12, h3, v0, e0, f0);
        h0p->setNeighbors(h1, h3p, v4, e0p, f0p);
        h1->setNeighbors(h10, h6, v2, h1->edge(), f0p);
        h2->setNeighbors(h0, h7, v1, h2->edge(), f0);
        h3->setNeighbors(h4, h0, v4, e0, f1);
        h3p->setNeighbors(h11, h0p, v2, e0p, f1p);
        h4->setNeighbors(h13, h8, v0, h4->edge(), f1);
        h5->setNeighbors(h3p, h9, v3, h5->edge(), f1p);
        h10->setNeighbors(h0p, h12, v1, e5, f0p);
        h11->setNeighbors(h5, h13, v4, e6, f1p);
        h12->setNeighbors(h2, h10, v4, e5, f0);
        h13->setNeighbors(h3, h11, v3, e6, f1);

        e5->isNew = true;
        e6->isNew = true;

        v4->position = .5*v0->position + .5*v2->position;
        return v4;
    }


    void MeshResampler::upsample(HalfedgeMesh &mesh) {
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


        for (auto v = mesh.verticesBegin(); v != mesh.verticesEnd(); v ++) {
            v->isNew = false;
            auto h = v->halfedge();
            auto hp = h;
            int degree = 0;
            Vector3D sum;
            do {
                degree ++;
                sum += hp->next()->vertex()->position;
                hp = hp->twin()->next();
            } while(hp != h);
            double u = (degree==3)?3.0/8.0 : 3.0/8.0/(double)degree;
            v->newPosition = (1-degree*u)*v->position + u*sum;
        }
        std::vector<EdgeIter> edges;
        for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); e ++) {
            edges.push_back(e);

        }
        for (auto e: edges){
            e->isNew = false;
            Vector3D pos;
            auto h = e->halfedge();
            pos += 3.0/8.0* h->vertex()->position;
            pos += 3.0/8.0* h->twin()->vertex()->position;
            pos += 1.0/8.0* h->next()->next()->vertex()->position;
            pos += 1.0/8.0* h->twin()->next()->next()->vertex()->position;
            e->newPosition = pos;
        }
        edges.clear();
        for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); e ++) {
            edges.push_back(e);

        }
        for (auto e :edges) {
            auto v = mesh.splitEdge(e);
            v->newPosition = e->newPosition;
            v->isNew = true;
        }
        edges.clear();
        for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); e ++) {
            edges.push_back(e);

        }
        for (auto e :edges) {
            if (!e->isNew) {
                continue;
            }
            if (e->halfedge()->vertex()->isNew != e->halfedge()->twin()->vertex()->isNew) {
                mesh.flipEdge(e);
            }
        }

        for (auto v = mesh.verticesBegin(); v != mesh.verticesEnd(); v ++) {
            v->position = v->newPosition;
        }
    }
}
