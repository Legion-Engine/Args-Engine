#include <physics/halfedgeface.hpp>
#include <physics/halfedgeedge.hpp>
#include <rendering/debugrendering.hpp>

namespace legion::physics
{
    HalfEdgeFace::HalfEdgeFace(HalfEdgeEdge* newStartEdge, math::vec3 newNormal) : startEdge{ newStartEdge }, normal{ newNormal }
    {
        /*log::debug("HalfEdgeFace::HalfEdgeFace");*/
        static int faceCount = 0;

        DEBUG_color =math::color( math::linearRand(0.25f, 0.7f), math::linearRand(0.25f, 0.7f), math::linearRand(0.25f, 0.7f));
        math::vec3 faceCenter{ 0.0f };
        int edgeCount = 0;

        auto calculateFaceCentroid = [&faceCenter, &edgeCount](HalfEdgeEdge* edge)
        {
            math::vec3 pos = edge->edgePosition;
            faceCenter += pos;
            edgeCount++;
        };
        forEachEdge(calculateFaceCentroid);

        centroid = faceCenter / static_cast<float>(edgeCount);

        int currentEdgeId = 0;

        auto initializeEdgeToFaceFunc = [this, &currentEdgeId, edgeCount](HalfEdgeEdge* edge)
        {
            edge->face = this;

            int nextID = currentEdgeId + 1 < edgeCount ? currentEdgeId + 1 : 0;

            EdgeLabel label
            (std::make_pair(faceCount, currentEdgeId), std::make_pair(faceCount, nextID));

            edge->label = std::move(label);

            currentEdgeId++;
        };

        forEachEdge(initializeEdgeToFaceFunc);

        faceCount++;

    }

    void HalfEdgeFace::deleteEdges()
    {
        HalfEdgeEdge* current = startEdge->nextEdge;
        do
        {
            if (current->prevEdge->pairingEdge && current->prevEdge->pairingEdge->pairingEdge == current) current->prevEdge->pairingEdge->pairingEdge = nullptr;
            delete current->prevEdge;
            current = current->nextEdge;
        } while (current != startEdge && current != nullptr);
        startEdge = nullptr;
    }

    void HalfEdgeFace::setFaceForAllEdges()
    {
        HalfEdgeEdge* current = startEdge;
        do
        {
            current->face = this;
            current = current->nextEdge;
        } while (current != startEdge);
    }


    void HalfEdgeFace::forEachEdge(legion::core::delegate< void(HalfEdgeEdge*)> functionToExecute)
    {
        HalfEdgeEdge* initialEdge = startEdge;
        HalfEdgeEdge* currentEdge = startEdge;

        if (!currentEdge) return;

        //the HalfEdgeEdge* 'startEdge' creates a ring buffer.
        //This means that initialEdge will eventually go back to "startEdge", ending the loop.
        do
        {
            HalfEdgeEdge* edgeToExecuteOn = currentEdge;
            currentEdge = currentEdge->nextEdge;
            functionToExecute(edgeToExecuteOn);

        } while (initialEdge != currentEdge && currentEdge->nextEdge != nullptr);

    }

    void HalfEdgeFace::inverse()
    {
        normal = -normal; // Inverse the normal
    }

    bool HalfEdgeFace::testConvexity(const HalfEdgeFace& other) const
    {
        if (other == *this)
        {
            log::warn("Testing face with itself for convexity: returning true");
            return true;
        }
        math::vec3 difference = startEdge->edgePosition - other.centroid;
        float scaledAngle = math::dot(difference, normal);

        // if the scaledAngle is smaller or equal to 0, it is not convex
        if (scaledAngle <= 0)
        {
            return false;
        }

        return true;
    }

    bool HalfEdgeFace::makeNormalsConvexWithFace(HalfEdgeFace& other)
    {
        if (other == *this)
        {
            log::warn("Make normals for face convex with itself: returning false");
            return false;
        }

        face_angle_relation relation= other.getAngleRelation(*this);

        if (relation == face_angle_relation::concave)
        {
            inverse();
            return true;
        }

        if (relation == face_angle_relation::coplanar)
        {
            log::warn("COPLANAR");
            this->normal = other.normal;
            return true;

        }

        //math::vec3 difference = startEdge->edgePosition - other.centroid;
        //float scaledAngle = math::dot(difference, normal);

        //// if the scaledAngle is smaller or equal to 0, it is not convex
        //if (scaledAngle < -math::epsilon<float>())
        //{
        //    inverse();
        //    return true;
        //}
        //else if (scaledAngle < math::epsilon<float>())
        //{
        //    this->normal = other.normal;
        //    log::warn("COPLANAR");
        //    return true;
        //}
        return false;
    }

    bool HalfEdgeFace::makeNormalsConvexWithNeighbors(HalfEdgeFace& other)
    {
        HalfEdgeEdge* start = this->startEdge;
        HalfEdgeEdge* end = nullptr;

       //do
       // {
       //     log::debug(":makeNormalsConvexWithNeighbors 1 ");
       //     if (start == other.startEdge->pairingEdge)
       //     {
       //         end = start;
       //         break;
       //     }

       //     start = start->nextEdge;
       // }
       // while (start != this->startEdge);
        end = HalfEdgeFace::findMiddleEdge(*this, other);

        if (end == nullptr)
        {
            //log::debug("end == nullptr ");
            return false;
        }

        start = end;
       // end = this->startEdge;

        do
        {
            //log::debug(":makeNormalsConvexWithNeighbors 2 ");

            HalfEdgeFace* face = start->pairingEdge->face;
            face_angle_relation relation = face->getAngleRelation(*this);

            if (relation == face_angle_relation::concave)
            {
                inverse();
                //log::debug("inversing normals");
                return true;
            }
            else if (relation == face_angle_relation::convex)
            {
                return false;
            }

            //if (relation == face_angle_relation::coplanar)
            //{
            //   // log::debug("COPLANAR");
            //    this->normal = other.normal;
            //    return true;

            //}
            start = start->nextEdge;
        } while (start != end);

        //log::debug("mission failed");
        return false;
    }

    HalfEdgeFace::face_angle_relation HalfEdgeFace::getAngleRelation(const HalfEdgeFace& other)
    {
        if (other == *this)
        {
            log::warn("Calculating face angle relation between the same face, returning coplanar");
            return face_angle_relation::coplanar;
        }
  
        //log::debug("IN FUNCTION face i {} face j {}", math::to_string(normal), math::to_string(other.normal));
        float distToPlane = math::pointToPlane(centroid, other.centroid, other.normal);
        //float distToPlane2 = math::pointToPlane(other.centroid, centroid, normal);
        /*log::debug("centroid {} other.centroid {} ", centroid, other.centroid);
        log::debug("distToPlane {} ", distToPlane);
        log::debug("distToPlane2 {} ", distToPlane2);
        log::debug("angle {} ", math::angleToPlane(centroid, other.centroid, other.normal));
        log::debug("angle2 {} ", math::angleToPlane(other.centroid, centroid, normal));*/
        //distToPlane2 = distToPlane;
        //log::debug("\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\");

        static float error = math::sqrt(math::epsilon<float>()) ;
       
        // if the distance to the face is negative, is it under the other face, therefore convex
        if (distToPlane <= -error)
        {
            return face_angle_relation::convex;
        }
        else if (distToPlane >= error)
        {
            return face_angle_relation::concave;
        }
        else return face_angle_relation::coplanar;
    }

    bool HalfEdgeFace::testConvexity(const HalfEdgeFace& first, const HalfEdgeFace& second)
    {
        if (first == second)
        {
            log::warn("Testing face with itself for convexity: returning true");
            return true;
        }
        return first.testConvexity(second) && second.testConvexity(first);
    }

    bool HalfEdgeFace::makeNormalsConvexWithFace(HalfEdgeFace& first, HalfEdgeFace& second)
    {
        if (first == second)
        {
            log::warn("Make normals for face convex with itself: returning false");
            return false;
        }
        bool inversedNormal = first.makeNormalsConvexWithFace(second);
        inversedNormal |= second.makeNormalsConvexWithFace(first);
        return inversedNormal;
    }

     void HalfEdgeFace::mergeCoplanarNeighbors(std::vector<HalfEdgeFace*> & removed)
     {

        HalfEdgeEdge* current = startEdge;
        if (this->normal != math::vec3(0, 0, 0))
        {
            return;
        }

        do
        {
            HalfEdgeFace* neigbor = current->pairingEdge->face;

            if (neigbor->normal != math::vec3(0, 0, 0))
            {
                current = current->nextEdge;
                continue;
            }
            face_angle_relation relation = this->getAngleRelation(*neigbor);

            if (relation == face_angle_relation::coplanar)
            {
                //log::debug("relation == face_angle_relation::coplanar in :mergeCoplanarNeighbors ");
                HalfEdgeEdge* middle = HalfEdgeFace::findMiddleEdge(*this, *neigbor);
                if (middle)
                {
                    this->mergeFaces(*middle);

                    removed.push_back(neigbor);
                    neigbor->normal = math::vec3();
                }
           
            }

           // this->mergeFaces
            current = current->nextEdge;
        }
        while (current != startEdge);

        return;
    }



    HalfEdgeEdge* HalfEdgeFace::findMiddleEdge(const HalfEdgeFace& first, const HalfEdgeFace& second)
    {
        HalfEdgeEdge* firstCurrent = first.startEdge;
        do
        {
            HalfEdgeEdge* secondCurrent = second.startEdge;
            do
            {
                if (firstCurrent->pairingEdge == secondCurrent)
                {
                    return firstCurrent;
                }
                if (firstCurrent == secondCurrent->pairingEdge)
                {
                    return firstCurrent;
                }


                secondCurrent = secondCurrent->nextEdge;
            } while (secondCurrent != second.startEdge);
            firstCurrent = firstCurrent->nextEdge;
        } while (firstCurrent != first.startEdge);

        return nullptr;
    }

    HalfEdgeFace* HalfEdgeFace::mergeFaces(HalfEdgeEdge& middleEdge)
    {
        if (middleEdge.face->startEdge == &middleEdge)
        {
            middleEdge.face->startEdge = middleEdge.face->startEdge->prevEdge;
        }
        // Set correct faces
        HalfEdgeEdge* end = middleEdge.pairingEdge;
        HalfEdgeEdge* current = middleEdge.pairingEdge;
        do
        {
            current->face = middleEdge.face;
            current = current->nextEdge;
        } while (current != end);

        // Link edges
        middleEdge.prevEdge->nextEdge = middleEdge.pairingEdge->nextEdge;
        middleEdge.nextEdge->prevEdge = middleEdge.pairingEdge->prevEdge;
        middleEdge.pairingEdge->prevEdge->nextEdge = middleEdge.nextEdge;
        middleEdge.pairingEdge->nextEdge->prevEdge = middleEdge.prevEdge;

        HalfEdgeFace* face = middleEdge.face;

        // Recalculate centroid
        {
            math::vec3 faceCenter{ 0.0f };
            int edgeCount = 0;

            auto calculateFaceCentroid = [&faceCenter, &edgeCount](HalfEdgeEdge* edge)
            {
                math::vec3 pos = edge->edgePosition;
                faceCenter += pos;
                edgeCount++;
            };
            face->forEachEdge(calculateFaceCentroid);

            face->centroid = faceCenter / static_cast<float>(edgeCount);
        }

        delete middleEdge.pairingEdge;
        delete& middleEdge;

        return face;
    }

    void HalfEdgeFace::DEBUG_DrawFace(const math::mat4& transform,const math::color& debugColor,float time)
    {

        auto drawFunc = [&transform,debugColor,time](HalfEdgeEdge* edge)
        {
            edge->DEBUG_drawEdge(transform, debugColor, time);

        };

        forEachEdge(drawFunc);
    }

    HalfEdgeFace::~HalfEdgeFace()
    {
        auto deleteFunc = [](HalfEdgeEdge* edge)
        {
            if (edge->pairingEdge && edge->pairingEdge && edge->pairingEdge->pairingEdge == edge)
            {
                edge->pairingEdge->pairingEdge = nullptr;
            }
            delete edge;
        };

        forEachEdge(deleteFunc);

    }
}
