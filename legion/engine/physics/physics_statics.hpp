#pragma once
#include <core/core.hpp>
#include <physics/colliders/convexcollider.hpp>
#include <physics/data/pointer_encapsulator.hpp>
#include <physics/data/contact_vertex.hpp>
#include <Voro++/voro++.hh>
#include <rendering/debugrendering.hpp>
#include <physics/data/convex_convex_collision_info.hpp>

namespace legion::physics
{
    struct HalfEdgeFace;

    class PhysicsStatics
    {
    public:
        //TODO move implementation to seperate cpp file

        //---------------------------------------------------------------- Collision Detection ----------------------------------------------------------------------------//

        static void DetectConvexConvexCollision(ConvexCollider* convexA, ConvexCollider* convexB
            , const math::mat4& transformA, const math::mat4& transformB,
            ConvexConvexCollisionInfo& outCollisionInfo,  physics_manifold& manifold);

        /** @brief Given a transformed ConvexCollider and a direction, Gets the vertex furthest in the given direction
         * @param planePosition The position of the support plane in world space
         * @param direction The direction we would like to know the support point of
         * @param collider The ConvexCollider in question
         * @param colliderTransform A mat4 describing the transform of the collider
         * @param worldSupportPoint [out] the resulting support point
         * @return returns true if a seperating axis was found
         */
        static void GetSupportPoint(const math::vec3& planePosition, const math::vec3& direction, ConvexCollider* collider, const math::mat4& colliderTransform
            , math::vec3& worldSupportPoint)
        {
            float largestDistanceInDirection = std::numeric_limits<float>::lowest();

            for (const auto& vert : collider->GetVertices())
            {
                math::vec3 transformedVert = colliderTransform * math::vec4(vert, 1);

                float dotResult = math::dot(transformedVert - planePosition, direction);

                if (dotResult > largestDistanceInDirection)
                {
                    largestDistanceInDirection = dotResult;
                    worldSupportPoint = transformedVert;
                }
            }



        }

        static float GetSupportPoint(const std::vector<math::vec3>& vertices, const math::vec3& direction, math::vec3& outVec);
        
        /** @brief Given 2 ConvexColliders, convexA and convexB, checks if one of the faces of convexB creates a seperating axis
         * that seperates the given convex shapes
         * @param convexA the reference collider
         * @param convexB the collider that will create the seperating axes
         * @param transformA the transform of convexA
         * @param transformB the transform of convexB
         * @param refFace [out] a HalfEdgeEdge* that has a normal parallel to the seperating axis
         * @param maximumSeperation [out] the seperation on the given seperating axis
         * @return returns true if a seperating axis was found
         */
        static bool FindSeperatingAxisByExtremePointProjection(ConvexCollider* convexA
            , ConvexCollider* convexB, const math::mat4& transformA, const math::mat4& transformB, PointerEncapsulator<HalfEdgeFace>&refFace, float& maximumSeperation)
        {
            float currentMaximumSeperation = std::numeric_limits<float>::lowest();

            for (auto face : convexB->GetHalfEdgeFaces())
            {

                //og::debug("face->normal {} ", math::to_string( face->normal));
                //get inverse normal
                math::vec3 seperatingAxis = math::normalize(transformB * math::vec4(math::normalize(face->normal), 0));

                math::vec3 transformedPositionB = transformB * math::vec4(face->centroid, 1);

                //get extreme point of other face in normal direction
                math::vec3 worldSupportPoint;
                GetSupportPoint(transformedPositionB, -seperatingAxis,
                    convexA, transformA, worldSupportPoint);

                float seperation = math::dot(worldSupportPoint - transformedPositionB, seperatingAxis);

                if (seperation > currentMaximumSeperation)
                {
                    currentMaximumSeperation = seperation;
                    refFace.ptr = face;
                }

                if (seperation > 0)
                {
                    //we have found a seperating axis, we can exit early
                    maximumSeperation = currentMaximumSeperation;
                    return true;
                }
            }
            //no seperating axis was found
            maximumSeperation = currentMaximumSeperation;

            return false;
        }

        /** @brief Given 2 ConvexColliders, Goes through every single possible edge combination in order to check for a valid seperating axis. This is done
         * by creating a minkowski face with each edge combination.
         * @param convexA the reference collider
         * @param convexB the incident collider
         * @param transformA the transform of convexA
         * @param transformB the transform of convexB
         * @param refEdge [out] the resulting reference Edge
         * @param incEdge [out] the resulting incident Edge
         * @param seperatingAxisFound [out] the resulting seperating axis found
         * @param seperation [out] the amount of seperation
         * @return returns true if a seperating axis was found
         */
        static bool FindSeperatingAxisByGaussMapEdgeCheck(ConvexCollider* convexA, ConvexCollider* convexB,
            const math::mat4& transformA, const math::mat4& transformB, PointerEncapsulator<HalfEdgeEdge>& refEdge, PointerEncapsulator<HalfEdgeEdge>& incEdge,
            math::vec3& seperatingAxisFound, float& maximumSeperation)
        {
            float currentMinimumSeperation = std::numeric_limits<float>::max();

            math::vec3 centroidDir = transformA * math::vec4(convexA->GetLocalCentroid(), 0);
            math::vec3 positionA = math::vec3(transformA[3]) + centroidDir;

            for (const auto faceA : convexA->GetHalfEdgeFaces())
            {
                //----------------- Get all edges of faceA ------------//
                std::vector<HalfEdgeEdge*> convexAHalfEdges;

                auto lambda = [&convexAHalfEdges](HalfEdgeEdge* edge)
                {
                    convexAHalfEdges.push_back(edge);
                };

                faceA->forEachEdge(lambda);

                for (const auto faceB : convexB->GetHalfEdgeFaces())
                {
                    //----------------- Get all edges of faceB ------------//
                    std::vector<HalfEdgeEdge*> convexBHalfEdges;

                    auto lambda = [&convexBHalfEdges](HalfEdgeEdge* edge)
                    {
                        convexBHalfEdges.push_back(edge);
                    };

                    faceB->forEachEdge(lambda);
      
                    for (HalfEdgeEdge* edgeA : convexAHalfEdges)
                    {
                        for (HalfEdgeEdge* edgeB : convexBHalfEdges)
                        {
                            //if the given edges creates a minkowski face
                            if (attemptBuildMinkowskiFace(edgeA, edgeB, transformA, transformB))
                            {
                                //get world edge direction
                                math::vec3 edgeADirection = transformA * math::vec4(edgeA->nextEdge->edgePosition, 1) - 
                                    transformA * math::vec4(edgeA->edgePosition, 1);


                                math::vec3 edgeBDirection = transformB * math::vec4(edgeB->nextEdge->edgePosition, 1) -
                                    transformB * math::vec4(edgeB->edgePosition, 1);

                                edgeADirection = math::normalize(edgeADirection);
                                edgeBDirection = math::normalize(edgeBDirection);

                                //get the seperating axis
                                math::vec3 seperatingAxis = math::cross(edgeADirection, edgeBDirection);

                                if (math::epsilonEqual(math::length(seperatingAxis), 0.0f, math::epsilon<float>()))
                                {
                                    continue;
                                }

                                seperatingAxis = math::normalize(seperatingAxis);

                                //get world edge position
                                math::vec3 edgeAtransformedPosition = transformA * math::vec4(edgeA->edgePosition, 1);
                                math::vec3 edgeBtransformedPosition = transformB * math::vec4(edgeB->edgePosition, 1);

                                //check if its pointing in the right direction 
                                if (math::dot(seperatingAxis, edgeAtransformedPosition - positionA) < 0)
                                {
                                    seperatingAxis = -seperatingAxis;
                                }

                                //check if given edges create a seperating axis
                                float distance = math::dot(seperatingAxis, edgeBtransformedPosition - edgeAtransformedPosition);
                                //log::debug("distance {} , currentMinimumSeperation {}", distance, currentMinimumSeperation);
                                if (distance < currentMinimumSeperation)
                                {                                 
                                    refEdge.ptr = edgeA;
                                    incEdge.ptr = edgeB;

                                    seperatingAxisFound = seperatingAxis;
                                    currentMinimumSeperation = distance;
                                }
                                //log::debug("BUILT MINKOWSKI");
                            }
                            else
                            {
                                //log::debug("NOT BUILT");
                            }
                        }
                    }
                }
            }
            /*assert(refEdge.ptr);
            assert(incEdge.ptr);*/
            //log::debug("a id  {}  b id {} combination {} ", std::get<0>(ids), std::get<1>(ids), std::get<2>(ids));
            //refEdge.ptr->DEBUG_drawEdge(transformA, math::colors::red);
            //incEdge.ptr->DEBUG_drawEdge(transformB, math::colors::red);
            maximumSeperation = currentMinimumSeperation;
            return currentMinimumSeperation > 0.0f;
        }


        static std::tuple< math::vec3,math::vec3> ConstructAABBFromPhysicsComponentWithTransform
        (ecs::component_handle<physicsComponent> physicsComponentToUse, const math::mat4& transform);

        static float GetPhysicsComponentSupportPointAtDirection(math::vec3 direction,physicsComponent& physicsComponentToUse);

        static std::tuple< math::vec3, math::vec3> ConstructAABBFromVertices(const std::vector<math::vec3>& vertices);

        static std::tuple< math::vec3, math::vec3> ConstructAABBFromTransformedVertices(const std::vector<math::vec3>& vertices,const math::mat4& transform);

        //---------------------------------------------------------- Polyhedron Clipping ----------------------------------------------------------------------------//

        /** @brief Given a 3D plane, clips the vertices in the inputList and places the results in the output list
         *
         */
        static void SutherlandHodgmanFaceClip( math::vec3& planeNormal, math::vec3& planePosition,
            std::vector<ContactVertex>& inputList, std::vector<ContactVertex>& outputList, HalfEdgeEdge* clippingEdge)
        {
            for (size_t i = 0; i < inputList.size(); i++)
            {
                ContactVertex& currentVertex = inputList.at(i);
                ContactVertex& nextVertex = i + 1 >= inputList.size() ? inputList.at(0) : inputList.at(i + 1);


                float currentVertDistFromPlane = PointDistanceToPlane(planeNormal, planePosition, currentVertex.position);
                float nextVertDistFromPlane = PointDistanceToPlane(planeNormal, planePosition, nextVertex.position);

                //at a certain threshold, the vertices can be considered to be on the plane for numerical robustness reasons
                if (math::abs(currentVertDistFromPlane) < constants::sutherlandHodgmanClippingThreshold
                    || math::abs(nextVertDistFromPlane) < constants::sutherlandHodgmanClippingThreshold)
                {
                    outputList.push_back(nextVertex);
                    continue;
                }

                bool isCurrentVertexUnderPlane = currentVertDistFromPlane < 0.0f;
                bool isNextVertexUnderPlane = nextVertDistFromPlane < 0.0f;

                //we always check clipping with a line that goes from the point below the plane to the point outside the plane.
                //We do this mostly for numerical robustness reasons.
                const math::vec3& pointAbovePlane = isCurrentVertexUnderPlane ? nextVertex.position : currentVertex.position;
                const math::vec3& pointBelowPlane = isCurrentVertexUnderPlane ? currentVertex.position : nextVertex.position;

                if (isCurrentVertexUnderPlane && isNextVertexUnderPlane)
                {
                    //send next vertex to outputlist
                    outputList.push_back(nextVertex);
                }
                else if (!isCurrentVertexUnderPlane && isNextVertexUnderPlane)
                {
                    //send next vertex to outputlist and the intersection between the line and the plane

                    math::vec3 intersectionPoint;

                    if (FindLineToPlaneIntersectionPoint(planeNormal, planePosition,
                        pointBelowPlane, pointAbovePlane, intersectionPoint))
                    {
                        auto currentVertexLabel = currentVertex.label;
                        auto nextVertexLabel = clippingEdge->label;

                        EdgeLabel label(currentVertexLabel.firstEdge, nextVertexLabel.nextEdge);
                        outputList.push_back(ContactVertex(intersectionPoint, label));
                    }

                    outputList.push_back(nextVertex);

                }
                else if (isCurrentVertexUnderPlane && !isNextVertexUnderPlane)
                {
                    //send intersection between the line and the plane

                    math::vec3 intersectionPoint;

                    if (FindLineToPlaneIntersectionPoint(planeNormal, planePosition,
                        pointBelowPlane, pointAbovePlane, intersectionPoint))
                    {
                        auto currentVertexLabel = currentVertex.label;
                        auto nextVertexLabel = clippingEdge->label;

                        EdgeLabel label(currentVertexLabel.firstEdge, nextVertexLabel.nextEdge);
                        outputList.push_back(ContactVertex(intersectionPoint, label));
                    }
                }
            }
        }

        //------------------------------------------------------------ helper functions -----------------------------------------------------------------------//

        /**@brief Given a set of points that represent 2 line segments, find a line segment that
        * reperesents the closest points between the 2 lines
        * @param p1 The start of the first line
        * @param p2 The end of the second line
        * @param p3 The start of the third line
        * @param p4 The end of the fourth line
        */
        static void FindClosestPointsToLineSegment(math::vec3 p1, math::vec3 p2,
            math::vec3 p3, math::vec3 p4,math::vec3& outp1p2, math::vec3& outp3p4)
        {
            //------------------find the interpolants of both lines that represent the closest points of the line segments-----------//

            //Given the 2 closest points of the given 2 line segments, L1(x) and L2(y), where L1 is the line created from the points
            //p1 and p2, and L2 is the line created from the points p3 and p4. and x and y are the interpolants, We know that these
            //2 points create a line that is perpendicular to both the lines of L1 and L2. Therefore:

            // (L1(x) - L2(y)) . (p2 - p1) = 0
            // (L1(x) - L2(y)) . (p4 - p3) = 0

            //these 2 function can be simplified into a linear system of 2 variables.

            // ax + cy = e
            // bx + dy = f

            //where a,b,c,d,e,f are equal to the following:

            float a, b, c, d, e, f;

            // ( [p2] ^ 2 - [p1] ^ 2) 
            a = math::pow2(p2.x - p1.x) +
                math::pow2(p2.y - p1.y) +
                math::pow2(p2.z - p1.z);

            // ([p4] - [p3])([p2] - [p1])
            b = (p4.x - p3.x) * (p2.x - p1.x) +
                (p4.y - p3.y) * (p2.y - p1.y) +
                (p4.z - p3.z) * (p2.z - p1.z);

            // ([p4] - [p3])([p2] - [p1])
            c = b;

            // ( [p4] ^ 2 - [p3] ^ 2) 
            d = (math::pow2(p4.x - p3.x) +
                math::pow2(p4.y - p3.y) +
                math::pow2(p4.z - p3.z));

            //([p3] - [p1])([p2] - [p1])
            e = (p3.x - p1.x) * (p2.x - p1.x) +
                (p3.y - p1.y) * (p2.y - p1.y) +
                (p3.z - p1.z) * (p2.z - p1.z);

            //([p4] - [p3])([p3] - [p1])
            f = (p4.x - p3.x) * (p3.x - p1.x) +
                (p4.y - p3.y) * (p3.y - p1.y) +
                (p4.z - p3.z) * (p3.z - p1.z);

            math::vec2 interpolantVector = LinearSystemCramerSolver2D(a, b, c, d, e, f);

            outp1p2 = p1 + (p2 - p1) * math::clamp(interpolantVector.x,0.0f,1.0f);
            outp3p4 = p3 + (p4 - p3) * math::clamp(interpolantVector.y,0.0f,1.0f);

        }


        /**Solves a linear system with 2 equations using cramers rule
        * @param a The value at a11 of the matrix
        * @param b The value at a21 of the matrix
        * @param c The value at a12 of the matrix
        * @param d The value at a22 of the matrix
        * @param e The result of the linear transformation in the x axis
        * @param f The result of the linear transformation in the y axis
        * @note a,b,c,d is a column major matrix
        */
        static math::vec2 LinearSystemCramerSolver2D(float a, float b, float c, float d, float e, float f)
        {
            //[ a c ] [x] [e]
            //[ b d ] [y] [f]

            float denom = (a * d) - (c * b);

            //[ e c ] 
            //[ f d ] 
            float x  = ((e * d) - (c * f)) / denom;

            //[ a e ]
            //[ b f ] 
            float y = ((a * f) - (e * b)) / denom;

            return math::vec2(x,-y);
        }

        /**@brief Finds the distance of a point given a 3D plane
        * Assumes the plane normal is normalized
        */
        static float PointDistanceToPlane(const math::vec3& planeNormal, const math::vec3& planePosition, const math::vec3& point)
        {
            return math::dot(point - planePosition, planeNormal);
        }

        /**@brief Checks if the distance of a point from a 3D plane is above zero
        */
        static bool IsPointAbovePlane(const math::vec3& planeNormal, const math::vec3& planePosition, const math::vec3& point)
        {
            return PointDistanceToPlane(planeNormal, planePosition, point) > 0.0f;
        }

        /**@brief Given a line going from start point to end point finds the intersection point of the line to a given 3D plane
        * @return Returns true if an intersection is found
        */
        static bool FindLineToPlaneIntersectionPoint(const math::vec3& planeNormal, const math::vec3& planePosition,
            const math::vec3& startPoint, const math::vec3& endPoint, math::vec3& intersectionPoint)
        {
            math::vec3 direction = endPoint - startPoint;

            if (math::epsilonEqual(math::dot(direction, planeNormal), 0.0f, math::epsilon<float>()))
            {
                return false;
            }

            float t = FindLineToPointInterpolant(startPoint, endPoint, planePosition, planeNormal);

            intersectionPoint = startPoint + direction * t;

            return true;
        }

        /**@brief Given a line going from start point to end point finds the intersection point of the line to a given 3D plane
        * @param [out] interpolant The calculated interpolant required to get from the startPoint to the endPoint
        * @return Returns true if an intersection is found
        */
        static bool FindLineToPlaneIntersectionPoint(const math::vec3& planeNormal, const math::vec3& planePosition,
            const math::vec3& startPoint, const math::vec3& endPoint, math::vec3& intersectionPoint,float& interpolant)
        {
            math::vec3 direction = endPoint - startPoint;

            if (math::epsilonEqual(math::dot(direction, planeNormal), 0.0f, math::epsilon<float>()))
            {
                return false;
            }

            interpolant = FindLineToPointInterpolant(startPoint, endPoint, planePosition, planeNormal);
            

            intersectionPoint = startPoint + direction * interpolant;

            return true;
        }



        /** Given a line going from a startPoint to and endPoint, finds the interpolant required to intersect a given 3D plane
        */
        static float FindLineToPointInterpolant(const math::vec3& startPoint, const math::vec3& endPoint, const math::vec3& planePosition,
            const math::vec3& planeNormal)
        {
            return math::dot(planePosition - startPoint, planeNormal) / math::dot(endPoint - startPoint, planeNormal);
        }




        /**Creates a Voronoi diagram based on the given parameters.
        * @param points A list of points these will serve as the points of the voronoi diagram.
        * @param xRange The min and max of the width of the voronoi diagram space.
        * @param yRange The min and max of the height of the voronoi diagram space.
        * @param zRange The min and max of the depth of the voronoi diagram space.
        * @param containerResolution The resolution of the contianer the voronoi diagram will be generated in.
        * @param xPeriodic A bool that decides whether x should be periodic.
        * @param yPeriodic A bool that decides whether y should be periodic.
        * @param zPeriodic A bool that decides whether z should be periodic.
        * @param initMem The initial memory amount.
        * @return A list of lists of vec4's
        */
        static std::vector<std::vector<math::vec4>> GenerateVoronoi(std::vector<math::vec3> points,math::vec2 xRange, math::vec2 yRange,
            math::vec2 zRange, math::vec3 containerResolution, bool xPeriodic = false, bool yPeriodic = false, bool zPeriodic = false, int initMem = 8)
        {
            return GenerateVoronoi(points,xRange.x,xRange.y,yRange.x,yRange.y,zRange.x,zRange.y,containerResolution.x,containerResolution.y,containerResolution.z,xPeriodic,yPeriodic,zPeriodic,initMem);
        }

        /**Creates a Voronoi diagram based on the given parameters.
        * @param points A list of points these will serve as the points of the voronoi diagram.
        * @param xMin The min of the width of the voronoi diagram space.
        * @param xMax The max of the width of the voronoi diagram space.
        * @param yMin The min of the height of the voronoi diagram space.
        * @param yMax The  max of the height of the voronoi diagram space
        * @param zMin The min of the depth of the voronoi diagram space.
        * @param zMax The max of the depth of the voronoi diagram space 
        * @param conResX The x resolution of the contianer the voronoi diagram will be generated in.
        * @param conResY The y resolution of the contianer the voronoi diagram will be generated in.
        * @param conResZ The z resolution of the contianer the voronoi diagram will be generated in.
        * @param xPeriodic A bool that decides whether x should be periodic.
        * @param yPeriodic A bool that decides whether y should be periodic.
        * @param zPeriodic A bool that decides whether z should be periodic.
        * @param initMem The initial memory amount.
        * @return A list of lists of vec4's
        */
        static std::vector<std::vector<math::vec4>> GenerateVoronoi(std::vector<math::vec3> points,const double xMin = -5, const double xMax = 5, const double yMin = -5, const double yMax = 5, const double zMin = -5, const double zMax = 5,const double conResX = 10,const double conResY = 10 , const double conResZ = 10,bool xPeriodic = false,bool yPeriodic = false,bool zPeriodic = false,int initMem = 8)
        {
            voro::container con(xMin, xMax, yMin, yMax, zMin, zMax, conResX, conResY, conResZ, xPeriodic, yPeriodic, zPeriodic, initMem);
            return GenerateVoronoi(con,points);
        }

        /**Creates a Voronoi diagram based on the given parameters.
        * @param con The container that hold the parameters and generates the voronoi diagram.
        * @param points A list of points these will serve as the points of the voronoi diagram.
        * @return A list of lists of vec4's
        */
        static std::vector<std::vector<math::vec4>> GenerateVoronoi(voro::container& con,std::vector<math::vec3> points)
        {
            int i = 0;
            for (math::vec3 point : points)
            {
                con.put(i,point.x,point.y,point.z);
                i++;
            }
            return GenerateVoronoi(con);
        }

        /**Creates a Voronoi diagram based on the given parameters.
        * @param con The container that hold the parameters and generates the voronoi diagram.
        * @return A list of lists of vec4's
        */
        static std::vector<std::vector<math::vec4>> GenerateVoronoi(voro::container& con)
        {
            con.draw_cells_json("assets/voronoi/output/cells.json");
            std::ifstream f("assets/voronoi/output/cells.json");
            return serialization::SerializationUtil::JSONDeserialize< std::vector<std::vector<math::vec4>>>(f);
        }

    private:

        /** @brief Given 2 HalfEdgeEdges and their respective transforms, transforms their normals and checks if they create a minkowski face
         * @return returns true if a minkowski face was succesfully constructed
         */
        static bool attemptBuildMinkowskiFace(HalfEdgeEdge* edgeA, HalfEdgeEdge* edgeB, const math::mat4& transformA,
            const math::mat4& transformB)
        {
            const math::vec3 transformedA1 = transformA * math::vec4(edgeA->getLocalNormal(), 0);
            const math::vec3 transformedA2 = transformA * math::vec4(edgeA->pairingEdge->getLocalNormal(), 0);
            const math::vec3 transformedEdgeDirectionA = transformA * math::vec4(edgeA->getLocalEdgeDirection(), 0);

            const math::vec3 transformedB1 = transformB * math::vec4(edgeB->getLocalNormal(), 0);
            const math::vec3 transformedB2 = transformB * math::vec4(edgeB->pairingEdge->getLocalNormal(), 0);
            const math::vec3 transformedEdgeDirectionB = transformB * math::vec4(edgeB->getLocalEdgeDirection(), 0);

            math::vec3 positionA = transformA * math::vec4(edgeA->edgePosition, 1);

            return isMinkowskiFace(transformedA1, transformedA2, -transformedB1, -transformedB2
                , transformedEdgeDirectionA, transformedEdgeDirectionB);
        }

        /** @brief Given 2 arcs, one that starts from transformedA1 and ends at transformedA2 and another arc
         * that starts at transformedB1 and ends at transformedB2, checks if the given arcs collider each other
         * @return returns true if the given arcs intersect
         */
        static bool isMinkowskiFace(const math::vec3& transformedA1, const math::vec3& transformedA2,
            const math::vec3& transformedB1, const math::vec3& transformedB2
            ,const math::vec3& planeANormal, const math::vec3& planeBNormal)
        {
            //------------------------ Check if normals created by arcA seperate normals of B --------------------------------------//
            //CBA
            float planeADotB1 = math::dot(planeANormal, transformedB1);
            //DBA
            float planeADotB2 = math::dot(planeANormal, transformedB2);

            float dotMultiplyResultA =
                planeADotB1 * planeADotB2;

            //log::debug("dotMultiplyResultA {}", dotMultiplyResultA);

            if (dotMultiplyResultA > 0.0f )
            {
                return false;
            }

            //------------------------ Check if normals created by arcB seperate normals of A --------------------------------------//

            //ADC
            float planeBDotA1 = math::dot(planeBNormal, transformedA1);
            //BDC
            float planeBDotA2 = math::dot(planeBNormal, transformedA2);

            float  dotMultiplyResultB = planeBDotA1 * planeBDotA2;

            //log::debug("dotMultiplyResultB {}", dotMultiplyResultB);

            if (dotMultiplyResultB > 0.0f )
            {
                return false;
            }

            //------------------------ Check if arcA and arcB are in the same hemisphere --------------------------------------------//

            float dotMultiplyResultAB = planeADotB1 * planeBDotA2;

            //log::debug("dotMultiplyResultAB {}", dotMultiplyResultAB);

            if (planeADotB1  * planeBDotA2  < 0.0f)
            {
                return false;
            }

            return true;
        }
    };
}

