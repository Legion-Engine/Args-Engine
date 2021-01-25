#pragma once
#include<core/core.hpp>
#include <rendering/debugrendering.hpp>
namespace legion::rendering
{
    template <typename ValueType>
    class Octree
    {
    private:
        enum Subsection : uint8
        {
            TopLeftFront,
            TopRightFront,
            BotRightFront,
            BotLeftFront,
            TopLeftBack,
            TopRightBack,
            BotRightBack,
            BotLeftBack
        };

    public:

        Octree() = default;
        Octree(const Octree&) = default;
        Octree(Octree&&) noexcept = default;
        Octree& operator=(const Octree&) = default;
        Octree& operator=(Octree&&) noexcept = default;
        Octree(size_type capacity, math::vec3 min, math::vec3 max, math::vec3 position)
            : m_capacity(capacity), m_min(min), m_max(max), m_position(position) {};

        Octree(size_type capacity, math::vec3 min, math::vec3 max)
            : m_capacity(capacity), m_min(min), m_max(max)
        {
            m_position = (min + max) * 0.5f;
        };

        void insertNode(ValueType item, const math::vec3& pos)
        {
            //check if there are no children 
            if (!m_children)
            {
                //store item if possible
                if (m_items.size() < m_capacity)
                {
                    m_items.push_back(std::make_pair(pos, item));
                    return;
                }
                //initialize children since capacity is full
                m_children = std::make_unique<std::array<Octree, 8>>();
                m_children->at(Subsection::TopLeftFront) = Octree<ValueType>(m_capacity, math::vec3(m_min.x, m_position.y, m_min.z), math::vec3(m_position.x, m_max.y, m_position.z));
                m_children->at(Subsection::TopRightFront) = Octree<ValueType>(m_capacity, math::vec3(m_position.x, m_position.y, m_min.z), math::vec3(m_max.x, m_max.y, m_position.z));
                m_children->at(Subsection::BotRightFront) = Octree<ValueType>(m_capacity, math::vec3(m_position.x, m_min.y, m_min.z), math::vec3(m_max.x, m_position.y, m_position.z));
                m_children->at(Subsection::BotLeftFront) = Octree<ValueType>(m_capacity, math::vec3(m_min.x, m_min.y, m_min.z), math::vec3(m_position.x, m_position.y, m_position.z));
                m_children->at(Subsection::TopLeftBack) = Octree<ValueType>(m_capacity, math::vec3(m_min.x, m_position.y, m_position.z), math::vec3(m_position.x, m_max.y, m_max.z));
                m_children->at(Subsection::TopRightBack) = Octree<ValueType>(m_capacity, math::vec3(m_position.x, m_position.y, m_position.z), math::vec3(m_max.x, m_max.y, m_max.z));
                m_children->at(Subsection::BotRightBack) = Octree<ValueType>(m_capacity, math::vec3(m_position.x, m_min.y, m_position.z), math::vec3(m_max.x, m_position.y, m_max.z));
                m_children->at(Subsection::BotLeftBack) = Octree<ValueType>(m_capacity, math::vec3(m_min.x, m_min.y, m_position.z), math::vec3(m_position.x, m_position.y, m_max.z));
           
            }
            //there are children, insert into child
            int childIndex = GetChildIndex(pos);
            m_children->at(childIndex).insertNode(item, pos);
        };
        void DrawTree()
        {
            debug::drawCube(math::vec3(m_min.x, m_min.y, m_min.z), math::vec3(m_max.x, m_max.y, m_max.z), math::colors::black);
            if (m_children)
            {
                //for all children draw tree if its not a leaf
                for (int i = 0; i < 8; i++)
                {
                    m_children->at(i).DrawTree();
                }
            }
        };

        int GetTreeDepth(int inputDepth = 0)
        {
            int depth = inputDepth;


            if (m_children)
            {
                //iterate child octants and get their largest depth
                int newDepth = -INT16_MAX;
                for (int i = 0; i < 8; i++)
                {
                    if (m_children->at(i).GetTreeDepth(depth + 1) > newDepth)  newDepth = m_children->at(i).GetTreeDepth(depth + 1);
                }
                return depth + newDepth;
            }
            return depth;
        }
        void GetData(int depth, std::vector<math::vec3>* Data)
        {
            //get data for current tree depth
            for (auto& [pos,item] : m_items)
            {
                Data->push_back(pos);
            }
            //check if this tree has children
            if (m_children)
            {
                //if depth 0 has not been reached go deeper and decrease depth
                if (depth > 0)
                {
                    for (int i = 0; i < 8; i++)
                    {
                        m_children->at(i).GetData(depth - 1, Data);
                    }
                }
            }
            return;
        }
        //gets data of the tree starting at defined depth
        void GetDataPair(int depth, std::vector <std::pair<math::vec3, ValueType>>* Data)
        {
            //get data for current tree depth
            for (auto& item : m_items)
            {
                Data->push_back(item);
            }
            //check if this tree has children
            if (m_children)
            {
                //if depth 0 has not been reached go deeper and decrease depth
                if (depth > 0)
                {
                    for (int i = 0; i < 8; i++)
                    {
                        m_children->at(i).GetDataPair(depth - 1, Data);
                    }
                }
            }
            return;
        }
        //gets data from starting depth until end depth
        void GetDataRangePair(int startingDepth, int endDepth, std::vector <std::pair<math::vec3, ValueType>>* Data)
        {
            //check if starting depth has been reached
            if (startingDepth > 0)
            {
                //if there are children go deeper
                if (m_children)
                {
                    for (int i = 0; i < 8; i++)
                    {
                        m_children->at(i).GetDataRangePair(startingDepth - 1, endDepth - 1, Data);
                    }
                }
            }
            else
            {
                //only continue if end depth has not been reached
                if (endDepth > 0)
                {
                    //start depth has been reached, end depth has not yet been reached, start getting data and go deeper
                    for (auto& item : m_items)
                    {
                        Data->push_back(item);
                    }
                    //if there are children go deeper 
                    if (m_children)
                    {
                        for (int i = 0; i < 8; i++)
                        {
                            m_children->at(i).GetDataRangePair(startingDepth, endDepth - 1, Data);
                        }
                    }
                }
            }
        }

        void GetDataRange(int startingDepth, int endDepth, std::vector<math::vec3>* Data)
        {
            //check if starting depth has been reached
            if (startingDepth > 0)
            {
                //if there are children go deeper
                if (m_children)
                {
                    for (int i = 0; i < 8; i++)
                    {
                        m_children->at(i).GetDataRange(startingDepth - 1, endDepth - 1, Data);
                    }
                }
            }
            else
            {
                //only continue if end depth has not been reached
                if (endDepth > 0)
                {
                    //start depth has been reached, end depth has not yet been reached, start getting data and go deeper
                    for (auto& [itemPos, item] : m_items)
                    {
                        Data->push_back(itemPos);
                    }
                    //if there are children go deeper 
                    if (m_children)
                    {
                        for (int i = 0; i < 8; i++)
                        {
                            m_children->at(i).GetDataRange(startingDepth, endDepth - 1, Data);
                        }
                    }
                }
            }
        }
    private:
        //checks for the index of the proper child quadrant
        int GetChildIndex(const math::vec3& pos)
        {
            if (pos.x < m_position.x)
            {
                if (pos.y > m_position.y)
                {
                    if (pos.z < m_position.z)
                        return Subsection::TopLeftFront;
                    else
                        return Subsection::TopLeftBack;
                }
                else
                {
                    if (pos.z < m_position.z)
                        return Subsection::BotLeftFront;
                    else
                        return Subsection::BotLeftBack;
                }
            }
            else
            {
                if (pos.y > m_position.y)
                {
                    if (pos.z < m_position.z)
                        return Subsection::TopRightFront;
                    else
                        return Subsection::TopRightBack;
                }
                else
                {
                    if (pos.z < m_position.z)
                        return Subsection::BotRightFront;
                    else
                        return Subsection::BotRightBack;
                }

            }
        }

        std::shared_ptr<std::array<Octree, 8>>  m_children;
        std::vector <std::pair<math::vec3, ValueType>>m_items;
        //capacity of octree level
        size_type m_capacity;
        //tree bounds
        math::vec3 m_min;
        math::vec3 m_max;
        //tree posiiton
        math::vec3 m_position;
    };
}
