#pragma once

#include <limits>
#include <core/core.hpp>
#include <physics/colliders/physicscollider.hpp>
#include <physics/cube_collider_params.hpp>
#include <physics/halfedgeedge.hpp>
#include <physics/halfedgeface.hpp>
#include <physics/data/convex_convergance_identifier.hpp>
#include <physics/data/physics_manifold.hpp>
#include <rendering/debugrendering.hpp>

namespace legion::physics
{
    class ConvexCollider : public PhysicsCollider
    {
    public:
        bool debug = false;
        int step = 0;
        ConvexCollider() = default;

        ~ConvexCollider()
        {
            for (auto face : halfEdgeFaces)
            {
                delete face;
            }
        }

        /** @brief Given a physics_contact that has been resolved, use its label and lambdas in order to create a ConvexConverganceIdentifier
       */
        void AddConverganceIdentifier(const physics_contact& contact) override
        {
            converganceIdentifiers.push_back(
                std::make_unique<ConvexConverganceIdentifier>(contact.label, contact.totalLambda,
                    contact.tangent1Lambda, contact.tangent2Lambda, GetColliderID()));
        }

        void CheckCollision(std::shared_ptr<PhysicsCollider> physicsCollider, physics_manifold& manifold) override
        {
            physicsCollider->CheckCollisionWith(this, manifold);
        }

        /** @brief Given a ConvexCollider and a physics_manifold, uses the seperating axis test for each HalfEdgeFace and edge
        * of both ConvexColliders
        */
        void CheckCollisionWith(ConvexCollider* convexCollider, physics_manifold& manifold) override;

        void PopulateContactPoints(std::shared_ptr<PhysicsCollider> physicsCollider, physics_manifold& manifold) override
        {
            physicsCollider->PopulateContactPointsWith(this, manifold);
        }

        void PopulateContactPointsWith(ConvexCollider* convexCollider, physics_manifold& manifold) override;

        void UpdateTransformedTightBoundingVolume(const math::mat4& transform) override
        {
            UpdateTightAABB(transform);
        }

        /**@brief Given the current transform of the entity, creates a tight AABB of the collider;
        */
        void UpdateTightAABB(const math::mat4& transform);
       

        /**@brief Using the vertices of the convexCollider,creates a ;
       */
        void UpdateLocalAABB() override;
       
        virtual void DrawColliderRepresentation(const math::mat4& transform, math::color usedColor, float width,float time,bool ignoreDepth = false) override;
        

        /**@brief Does one step of the convex hull generation
         * Just for debug purposes
         */
        void doStep(legion::core::mesh_handle& mesh)
        {
           
            ConstructConvexHullWithMesh(mesh);
            ++step;
        }

        void ConstructConvexHullWithVertices( std::vector<math::vec3>& vertices,math::vec3 spacingAmount = math::vec3())
        {
            //many of the functions for convex hull are coupled to mesh,
            //for now we create a mesh so that we can pass it into the function
            core::mesh mesh;
            mesh.vertices = std::move(vertices);
            ConstructConvexHullWithMesh(mesh,spacingAmount);
        }

        

        //TODO(algorythmix,jelled1st) This desperately needs cleanup
        //TODO(cont.) investigate unused variables! (projected)
        //LIKE A LOT OF CLEANUP

        /**@brief Constructs a polyhedron-shaped convex hull that encompasses the given mesh.
        * @param meshHandle - The mesh handle to lock the mesh and the mesh to create a hull from
        */
        void ConstructConvexHullWithMesh(legion::core::mesh_handle& meshHandle,bool shouldDebug = true)
        {
            auto meshLockPair = meshHandle.get();

            //mesh lock stuff
            async::readonly_guard guard(meshLockPair.first);
            auto mesh = meshLockPair.second;

            ConstructConvexHullWithMesh(mesh,math::vec3(),shouldDebug);
        }

        void  ConstructConvexHullWithMesh(mesh& mesh, math::vec3 spacingAmount = math::vec3(), bool shouldDebug = false);
       
        /**@brief Constructs a box-shaped convex hull that encompasses the given mesh.
        */
        void ConstructBoxWithMesh()
        {

        }

        void CalculateLocalColliderCentroid()
        {
            localColliderCentroid = math::vec3();

            for (auto& vertex : vertices)
            {
                localColliderCentroid += vertex;
            }
            
            localColliderCentroid /= vertices.size();
        }

        /**@brief Constructs a box-shaped convex hull based on the given parameters.
         * @param cubeParams The parameters that will be used to create the box.
        */
        void CreateBox(const cube_collider_params& cubeParams)
        {
            //initialize box vertices
            float halfWidth = cubeParams.width / 2;
            float halfBreath = cubeParams.breadth / 2;
            float halfHeight = cubeParams.height / 2;

            

            // BOTTOM FACE          // TOP FACE

            //a ----------- b       //e ----------- f
            //||           ||       //||           ||
            //||           ||       //||           ||
            //||           ||       //||           ||
            //||           ||       //||           ||                                              
            //c ----------- d       //g ----------- h

            /*a*/ math::vec3 minVertexPlusBreadth = math::vec3(-halfWidth, -halfHeight, halfBreath);
            /*b*/ math::vec3 minVertexPlusWidthPlusBreadth = math::vec3(halfWidth, -halfHeight, halfBreath);
            /*c*/ math::vec3 minVertex = math::vec3(-halfWidth, -halfHeight, -halfBreath);
            /*d*/ math::vec3 minVertexPlusWidth = math::vec3(halfWidth, -halfHeight, -halfBreath);

            /*e*/ math::vec3 maxVertexMinusWidth = math::vec3(-halfWidth, halfHeight, halfBreath);
            /*f*/ math::vec3 maxVertex = math::vec3(halfWidth, halfHeight, halfBreath);
            /*g*/ math::vec3 maxVertexMinusWidthMinusBreadth = math::vec3(-halfWidth, halfHeight, -halfBreath);
            /*h*/ math::vec3 maxVertexMinusBreadth = math::vec3(halfWidth, halfHeight, -halfBreath);

            //the HalfEdgeEdge only needs a ptr to the vertex  
            vertices.push_back(minVertexPlusBreadth);
            vertices.push_back(minVertexPlusWidthPlusBreadth);
            vertices.push_back(minVertex);
            vertices.push_back(minVertexPlusWidth);
            vertices.push_back(maxVertexMinusWidth);
            vertices.push_back(maxVertex);
            vertices.push_back(maxVertexMinusWidthMinusBreadth);
            vertices.push_back(maxVertexMinusBreadth);

            for (auto& vertex : vertices)
            {
                vertex += cubeParams.offset;
            }

            math::vec3 a = vertices.at(0);
            math::vec3 b = vertices.at(1);
            math::vec3 c = vertices.at(2);
            math::vec3 d = vertices.at(3);
            math::vec3 e = vertices.at(4);
            math::vec3 f = vertices.at(5);
            math::vec3 g = vertices.at(6);
            math::vec3 h = vertices.at(7);

            CalculateLocalColliderCentroid();

            //note: each edge carries adjacency information. (for example, an edge 'eg' must know its edge pair 'ge').
            //This is why each edge must be declared explicitly.

            //each face also has an id. It is mostly used for debugging reasons and will be removed when it is no longer needed.

            //[1] create face eghf

            HalfEdgeEdge* eg = new HalfEdgeEdge(e);
            HalfEdgeEdge* gh = new HalfEdgeEdge(g);
            HalfEdgeEdge* hf = new HalfEdgeEdge(h);
            HalfEdgeEdge* fe = new HalfEdgeEdge(f);

            eg->setNextAndPrevEdge(fe, gh);
            gh->setNextAndPrevEdge(eg, hf);
            hf->setNextAndPrevEdge(gh, fe);
            fe->setNextAndPrevEdge(hf, eg);

            HalfEdgeFace* eghf = new HalfEdgeFace(eg, math::vec3(0, 1, 0));
            halfEdgeFaces.push_back(eghf);
            //eghf->id = " eghf";

            //[2] create face hgcd

            HalfEdgeEdge* hg = new HalfEdgeEdge(h);
            HalfEdgeEdge* gc = new HalfEdgeEdge(g);
            HalfEdgeEdge* cd = new HalfEdgeEdge(c);
            HalfEdgeEdge* dh = new HalfEdgeEdge(d);

            hg->setNextAndPrevEdge(dh, gc);
            gc->setNextAndPrevEdge(hg, cd);
            cd->setNextAndPrevEdge(gc, dh);
            dh->setNextAndPrevEdge(cd, hg);

            HalfEdgeFace* hgcd = new HalfEdgeFace(hg, math::vec3(0, 0, -1));
            halfEdgeFaces.push_back(hgcd);
            //hgcd->id = "hgcd";

            //[3] create face fhdb

            HalfEdgeEdge* fh = new HalfEdgeEdge(f);
            HalfEdgeEdge* hd = new HalfEdgeEdge(h);
            HalfEdgeEdge* db = new HalfEdgeEdge(d);
            HalfEdgeEdge* bf = new HalfEdgeEdge(b);

            fh->setNextAndPrevEdge(bf, hd);
            hd->setNextAndPrevEdge(fh, db);
            db->setNextAndPrevEdge(hd, bf);
            bf->setNextAndPrevEdge(db, fh);

            HalfEdgeFace* fhdb = new HalfEdgeFace(fh, math::vec3(1, 0, 0));
            halfEdgeFaces.push_back(fhdb);
            //fhdb->id = "fhdb";

            //[4] create face efba

            HalfEdgeEdge* ef = new HalfEdgeEdge(e);
            HalfEdgeEdge* fb = new HalfEdgeEdge(f);
            HalfEdgeEdge* ba = new HalfEdgeEdge(b);
            HalfEdgeEdge* ae = new HalfEdgeEdge(a);

            ef->setNextAndPrevEdge(ae, fb);
            fb->setNextAndPrevEdge(ef, ba);
            ba->setNextAndPrevEdge(fb, ae);
            ae->setNextAndPrevEdge(ba, ef);

            HalfEdgeFace* efba = new HalfEdgeFace(ef, math::vec3(0, 0, 1));
            halfEdgeFaces.push_back(efba);
            //efba->id = "efba";

            //[5] create face geac

            HalfEdgeEdge* ge = new HalfEdgeEdge(g);
            HalfEdgeEdge* ea = new HalfEdgeEdge(e);
            HalfEdgeEdge* ac = new HalfEdgeEdge(a);
            HalfEdgeEdge* cg = new HalfEdgeEdge(c);

            ge->setNextAndPrevEdge(cg, ea);
            ea->setNextAndPrevEdge(ge, ac);
            ac->setNextAndPrevEdge(ea, cg);
            cg->setNextAndPrevEdge(ac, ge);

            HalfEdgeFace* geac = new HalfEdgeFace(ge, math::vec3(-1, 0, 0));
            halfEdgeFaces.push_back(geac);
            //geac->id = "geac";

            //[6] create face abdc

            HalfEdgeEdge* ab = new HalfEdgeEdge(a);
            HalfEdgeEdge* bd = new HalfEdgeEdge(b);
            HalfEdgeEdge* dc = new HalfEdgeEdge(d);
            HalfEdgeEdge* ca = new HalfEdgeEdge(c);

            ab->setNextAndPrevEdge(ca, bd);
            bd->setNextAndPrevEdge(ab, dc);
            dc->setNextAndPrevEdge(bd, ca);
            ca->setNextAndPrevEdge(dc, ab);

            HalfEdgeFace* abdc = new HalfEdgeFace(ab, math::vec3(0, -1, 0));
            halfEdgeFaces.push_back(abdc);
            //abdc->id = "abdc";


            //manually connect each edge to its pair

            //eghf                  //hgcd
            eg->pairingEdge = ge;   hg->pairingEdge = gh;
            gh->pairingEdge = hg;   gc->pairingEdge = cg;
            hf->pairingEdge = fh;   cd->pairingEdge = dc;
            fe->pairingEdge = ef;   dh->pairingEdge = hd;

            //fhdb                   //efba
            fh->pairingEdge = hf;    ef->pairingEdge = fe;
            hd->pairingEdge = dh;    fb->pairingEdge = bf;
            db->pairingEdge = bd;    ba->pairingEdge = ab;
            bf->pairingEdge = fb;    ae->pairingEdge = ea;

            //geac                  //abdc
            ge->pairingEdge = eg;   ab->pairingEdge = ba;
            ea->pairingEdge = ae;   bd->pairingEdge = db;
            ac->pairingEdge = ca;   dc->pairingEdge = cd;
            cg->pairingEdge = gc;   ca->pairingEdge = ac;


            //initialize the ID of the edges, this is done mostly for debugging reasons and will be removed when it
            //is no longer needed

            //eghf           //hgcd
            eg->id = "eg";   hg->id = "hg";
            gh->id = "gh";   gc->id = "gc";
            hf->id = "hf";   cd->id = "cd";
            fe->id = "fe";   dh->id = "dh";

            //fhdb            //efba
            fh->id = "fh";    ef->id = "ef";
            hd->id = "hd";    fb->id = "fb";
            db->id = "db";    ba->id = "ba";
            bf->id = "bf";    ae->id = "ae";

            //geac           //abdc
            ge->id = "ge";   ab->id = "ab";
            ea->id = "ea";   bd->id = "bd";
            ac->id = "ac";   dc->id = "dc";
            cg->id = "cg";   ca->id = "ca";

            //check if halfEdge data structure was initialized correctly. this will be commented when I know it always works
            AssertEdgeValidity();
            CalculateLocalColliderCentroid();
        }

        std::vector<HalfEdgeFace*>& GetHalfEdgeFaces() override
        {
            return halfEdgeFaces;
        }

        const std::vector<math::vec3>& GetVertices() const
        {
            return vertices;
        }

        void AssertEdgeValidity()
        {
            auto assertFunc = [](HalfEdgeEdge* edge)
            {
                assert(edge->nextEdge);
                assert(edge->prevEdge);
                assert(edge->pairingEdge);
                assert(edge->face);

            };

            for (auto& face : halfEdgeFaces)
            {
                face->forEachEdge(assertFunc);
            }
        }


    private:


        std::vector<math::vec3> vertices;

        HalfEdgeFace* instantiateMeshFace(const std::vector<math::vec3*>& vertices, const math::vec3& faceNormal)
        {
            if (vertices.size() == 0) { return nullptr; }

            std::vector<HalfEdgeEdge*> faceEdges;

            for (const auto vert : vertices)
            {
                faceEdges.push_back(new HalfEdgeEdge(*vert));
            }

            for (size_t i = 0; i < faceEdges.size(); i++)
            {
                int prevIndex = (i - 1) >= 0 ? (i - 1) : faceEdges.size() - 1;
                int afterIndex = (i + 1) < faceEdges.size() ? (i + 1) : 0;

                HalfEdgeEdge* prevEdge = faceEdges.at(prevIndex);
                HalfEdgeEdge* nextEdge = faceEdges.at(afterIndex);

                faceEdges.at(i)->setNextAndPrevEdge(prevEdge, nextEdge);

            }

            return new HalfEdgeFace(faceEdges.at(0), faceNormal);
        }

        /**@brief Function to find the outer two indices with the largest distance from the mesh
         * @brief The indices are found by constructing a virtual box around the mesh and checking all the outer vertices with each other for the greatest distance
         * @param mesh Mesh to find the outer vertices for. Make sure the mesh is locked (thread save)
         * @return std::pair<index_type, index_type> containing the outer vertices.
         */
        std::pair<index_type,index_type> convexHullFindOuterIndices(const core::mesh& mesh)
        {
            // Step 1 - create a box around the mesh

            math::vec3 first = mesh.vertices.at(0);
            // stores outer vertices in the following order:
            // min x, max x, min y, max y, min z, max z
            // The array is initialized with the first vertex
            float outer[] = { first.x, first.x, first.y, first.y, first.z, first.z };
            std::multimap<int, size_type> indicesMap;



            //lambda to find all indicies that match criteria
            auto comp = [&](size_type iterator, size_type index, auto op, size_type axis)
            {
                if (op(mesh.vertices.at(iterator)[axis], outer[index]))
                {
                    indicesMap.erase(index);
                    outer[index] = mesh.vertices.at(iterator)[axis];
                    indicesMap.emplace(index, iterator);
                }
                else if (mesh.vertices.at(iterator)[axis] == outer[index])
                {
                    indicesMap.emplace(index, iterator);
                }
            };


            for (size_type i = 1; i < mesh.vertices.size(); ++i)
            {
                //find all indices for all criteria (<x,>x,<y,>y,<z,>z)
                comp(i, 0, std::less<>(),    0);
                comp(i, 1, std::greater<>(), 0);
                comp(i, 2, std::less<>(),    1);
                comp(i, 3, std::greater<>(), 1);
                comp(i, 4, std::less<>(),    2);
                comp(i, 5, std::greater<>(), 2);
            }

            float largestDistance = 0;
            index_type index0;
            index_type index1;
            // Find the largest distance between two vertices in the multimap
            for(auto& [outer_key,outer_value] : indicesMap)
            {
                for(auto& [inner_key, inner_value] : indicesMap)
                {
                    if(inner_key == outer_key) continue;
                    float dist = distance(mesh.vertices[outer_value], mesh.vertices[inner_value]);

                    if( dist > largestDistance)
                    {
                        index0 = inner_value;
                        index1 = outer_value;
                        largestDistance = dist;
                    }
                }
            }
            return std::make_pair(index0, index1);
        }

        /**@brief Function to find the index in a mesh which is closest to a line between two vertices in the mesh
         * @param mesh The mesh with the vertices, make sure the mesh is locked (thread save)
         * @param lineStartIndex The index of the vertex where the line starts
         * @param lineEndIndex The index of the vertex at the end of the line
         * @return The index of the vertex with the largest distance from the line, mesh.vertices.size() is returned if a such a point cannot be found
         */
        size_type convexHullFindIndexClosestToLine(const core::mesh& mesh, const size_type lineStartIndex, const size_type lineEndIndex)
        {
            float largestDistance = 0;
            // Save the index which is furthest from the line between the mesh vertices at lineStartIndex and lineEndIndex
            size_type index = mesh.vertices.size();

            // Find the vertex in the mesh which has the greatest distance from the passed line

            // Loop through all vertices
            for (int i = 0; i < mesh.vertices.size(); ++i)
            {
                // Check if the index is not the start or end of line index
                if (i == lineStartIndex || i == lineEndIndex) continue;
                float dist = math::pointToLineSegment(mesh.vertices.at(i), mesh.vertices.at(lineStartIndex), mesh.vertices.at(lineEndIndex));
                if (dist > largestDistance && dist > 0)
                {
                    largestDistance = dist;
                    index = i;
                }
            }

            return index;
        }

        /**@brief Function to find the index in a mesh which is closest to the plane between triangle plane indices
         * @param mesh The mesh with the vertices, make sure that the mesh has been locked (thread save)
         * @param planeIndex0 First triangle plane vertex index
         * @param planeIndex1 Second triangle plane vertex index
         * @param planeIndex2 Third triangle plane vertex index
         * @param normal The normal of the plane vertex index
         * @return The index of the vertex with the largest distance from the plane, mesh.vertices.size() is returned if a such a point cannot be found
         */
        size_type convexHullFindIndexClosestToTriangle(const core::mesh& mesh, const size_type planeIndex0,
            const size_type planeIndex1, const size_type planeIndex2, const math::vec3 normal,bool debug = false)
        {
            float largestDistance = 0;
            // Save the index which is furthest from the plane between the mesh vertices at the plane indices
            int index = mesh.vertices.size();

            for (int i = 0; i < mesh.vertices.size(); ++i)
            {
                // If the index is the same as one of the triangle indices, continue
                if (i == planeIndex0 || i == planeIndex1 || i == planeIndex2) continue;

                if (math::abs(math::pointToPlane(mesh.vertices.at(i), mesh.vertices.at(planeIndex0), normal)) < math::epsilon<float>())
                {
                    continue;
                }

                float dist = abs(math::pointToTriangle(mesh.vertices.at(i), mesh.vertices.at(planeIndex0), mesh.vertices.at(planeIndex1), mesh.vertices.at(planeIndex2), normal,debug));
                if (dist > largestDistance && dist > 0)
                {
                    largestDistance = dist;
                    index = i;
                }
            }
            return index;
        }

        /**@brief Function to match unused mesh vertices to convex hull faces
         * @brief A vertex is matched to its closest face
         * @brief Vertices that are inside all faces are not stored
         * @param vertices The vertices that are to be matched
         * @return A list of a list of indeces in the vertices list.
         * std::vector<std::vector<int>>.at(index) gets the vector of vertices that are matched to face at targeted index in the halfEdgeFaces list
         * std::vector<int>.at(index) gets the vertex
         */
        std::vector<std::vector<math::vec3>> convexHullMatchVerticesToFace(const std::vector<math::vec3>& vertices)
        {
            std::vector<std::vector<math::vec3>> collection = std::vector<std::vector<math::vec3>>(halfEdgeFaces.size());

            for (const auto& vertex : vertices)
            {
                float smallestDistance = std::numeric_limits<float>::max();

                int faceIndex = -1;

                for (size_type f = 0; f < halfEdgeFaces.size(); ++f)
                {
                    // Get distance to face
                    float distToPlane = math::pointToPlane(
                        vertex, halfEdgeFaces.at(f)->startEdge->edgePosition, halfEdgeFaces.at(f)->normal);

                    // If the point is on or under the plane, the face cannot see the point,
                    // Therefore the point should not be matched with this face, and can be skipped
                    if (distToPlane > math::epsilon<float>())
                    {
                        //log::debug("vert distance {}", distToPlane);
                       
                        HalfEdgeEdge* startEdge = halfEdgeFaces.at(f)->startEdge;

                        // Get the current distance to the face
                        // Technically we should get the distance to a face, not to a triangle
                        float distance = math::pointToTriangle(
                            vertex, startEdge->edgePosition,
                            startEdge->nextEdge->edgePosition,
                            startEdge->prevEdge->edgePosition,
                            halfEdgeFaces.at(f)->normal
                        );

                        // Check if the distance is smaller and keep it if it is
                        if (distance > 0 && distance < smallestDistance)
                        {
                            faceIndex = f;
                            smallestDistance = distance;
                        }
                    }
                }
                // We check if we found a valid face
                // No valid faces can be found, when the vertex is inside the current hull
                if (faceIndex >= 0)
                {
                    // We push the vertex to the map of the face
                    collection[faceIndex].push_back(vertex);
                }
            }
            return collection;
        }

        /**
         * @brief Constructs a horizon from a vert view onto the current hull
         * 
         * @param vert The vert viewpoint
         * @param face The face from where the horizon starts, it is assumed this face is correct
         * @param edges The horizon edges, this deque is used as a return
         * @param originEdge The edge where the previous iteration came from, pass nullptr, only used for recursive calling
         * @param originFace The face where the origin started, pass nullptr, only used for recursive calling
         */
        void convexHullConstructHorizon(math::vec3 vert, HalfEdgeFace& face, std::deque<HalfEdgeEdge*>& edges,
            HalfEdgeEdge* originEdge = nullptr, std::shared_ptr<std::unordered_set<HalfEdgeFace*>> visited = nullptr);
        //TODO add documentation 
        void ConstructHorizonByEdgeJumping(math::vec3 vert, std::deque<HalfEdgeEdge*>& edges,
            std::vector<HalfEdgeFace*>& faces,math::vec3 spacing = math::vec3(),  bool shouldDebug =false);



       /**
        * @brief Merges coplanar faces between the passed vector of faces and the already existing faces in HalfEdgeFaces
        * 
        * @param faces The faces that might need to be merged with already existing faces
        * This vector of faces may be edited and faces in the HalfEdgeFaces might change when faces become merged 
        */
        void convexHullMergeFaces(std::vector<HalfEdgeFace*>& faces)
        {
            // Merge faces with coplanerity
            for (int i = faces.size() - 1; i >= 0; --i)
            {
                for (int j = 0; j < halfEdgeFaces.size(); ++j)
                {
                    if (faces.at(i) == halfEdgeFaces.at(j)) { continue; }
                     //Do not check if createdFace is halfEdgeFace, because the createdFaces are not yet added to the halfEdgeFaces list
                    //log::debug("face i {} face j {}", math::to_string(faces.at(i)->normal), math::to_string(halfEdgeFaces.at(j)->normal));
                    HalfEdgeFace::face_angle_relation relation = faces.at(i)->getAngleRelation(*(halfEdgeFaces.at(j)));
                    //log::debug("relation {}", HalfEdgeFace::to_string(relation));
                    


                    if (relation == HalfEdgeFace::face_angle_relation::coplanar)
                    {
                        // Try to find the edge between the faces that need to be merged
                        HalfEdgeEdge* centerEdge = HalfEdgeFace::findMiddleEdge(*halfEdgeFaces.at(j), *faces.at(i));
                        if (centerEdge != nullptr)
                        {
                            //log::debug("merge faces {}", halfEdgeFaces.at(j)->normal);


                            // Because we tried to get a center edge on the halfEdgeFaces.at(j) side
                            // That face should still exist and be correct
                            // We check if this is indeed the case: if the returned face is a different face
                            // we need to remove the old face and add the new face to the halfEdgeFaces vector
                            HalfEdgeFace* face = HalfEdgeFace::mergeFaces(*centerEdge);
                       
                            if (face != halfEdgeFaces.at(j))
                            {
                                halfEdgeFaces.erase(halfEdgeFaces.begin() + j);
                                halfEdgeFaces.push_back(face);
                            }
                            // Erase the face that was merged into the existing face from the faces vector
                            faces.erase(faces.begin() + i);
                            // We want to break because the face we were checking no longer exists
                            break;
                        }
                    }
                }
            }
        }

        
        std::vector<HalfEdgeFace*> halfEdgeFaces;

        // Convex hull generation debug stuffs
       
        std::vector<math::vec3> toBeSorted;
        std::vector<std::vector<math::vec3>> faceVertMap;
        std::unordered_map<HalfEdgeFace*, int> faceIndexMap;
        int looped = 0;

        //feature id container
    };
}


