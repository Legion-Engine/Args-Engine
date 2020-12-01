#pragma once
#include <core/math/glm/glm_include.hpp>

/**
 * @file geometry.hpp
 */

namespace legion::core::math
{
    /**@brief Calcualtes the shortest distance between a point and a line
     * @param point - The point
     * @param lineOrigin - The origin of the line
     * @param lineEnd - The end of the line
     */
    float pointToLine(vec3 point, vec3 lineOrigin, vec3 lineEnd)
    {
        vec3 dir = normalize(lineEnd - lineOrigin);
        vec3 toLineOrigin = lineOrigin - point;
        float distOnLine = dot(toLineOrigin, dir);
        vec3 closestPointOnLine = lineOrigin + (dir * distOnLine);
        return distance(closestPointOnLine, point);
    }

    /**@brief Calculates the shortest squared distance between a point and a line
     * @brief Squared distance removes the need for a sqrt calculation 
     * @param point - The point
     * @param lineOrigin - The origin of the line
     * @param lineEnd - The end of the line
     */
    float squaredDistanceToLine(vec3 point, vec3 lineOrigin, vec3 lineEnd)
    {
        vec3 dir = normalize(lineEnd - lineOrigin);
        vec3 toLineOrigin = lineOrigin - point;
        float distOnLine = dot(toLineOrigin, dir);
        vec3 closestPointOnLine = lineOrigin + (dir * distOnLine);
        return (closestPointOnLine.x * closestPointOnLine.x) + (closestPointOnLine.y * closestPointOnLine.y) + (closestPointOnLine.z * closestPointOnLine.z);
    }


    /**@class line_segment
     * @brief Data structure for a line segment
     */
    struct line_segment
    {
        line_segment(vec3 origin, vec3 end) :
            origin(origin), end(end)
        {
        }

        // Line origin, line start
        vec3 origin;
        // Line end
        vec3 end;

        inline vec3 direction()
        {
            return end - origin;
        }

        /**@brief Calculates the closest distance between point p and this line
         */
        float distanceToPoint(const vec3& p) const
        {
            vec3 dir = normalize(end - origin);
            vec3 toLineOrigin = origin - p;
            float distOnLine = dot(toLineOrigin, dir);
            vec3 closestPointOnLine = origin + (dir * distOnLine);
            return distance(closestPointOnLine, p);
        }

        /**@brief Calculates the closest squared distance between point p and this line
         * @brief Does not require a sqrt
         */
        float squaredDistanceToPoint(const vec3& p) const
        {
            vec3 dir = normalize(end - origin);
            vec3 toLineOrigin = origin - p;
            float distOnLine = dot(toLineOrigin, dir);
            vec3 closestPointOnLine = origin + (dir * distOnLine);
            return (closestPointOnLine.x * closestPointOnLine.x) + (closestPointOnLine.y * closestPointOnLine.y) + (closestPointOnLine.z * closestPointOnLine.z);
        }
    };

    /**@brief Calculates the size of a triangles surface area
     */
    float triangleSurface(vec3 p0, vec3 p1, vec3 p2)
    {
        return 0.5 * (normalizeDot(p0, p1) * normalizeDot(p0, p2)) * fastSin(0);
    }

    /**@brief Calculates the distance between a point and a triangle plane
     * @param p - The point
     * @param triPoint0 - The first triangle point
     * @param triPoint1 - The second triangle point
     * @param triPoint2 - The last triangle point
     * @param triNormal - The triangle plane normal
     */
    float pointToTriangle(const vec3& p, vec3 triPoint0, vec3 triPoint1, vec3 triPoint2, vec3 triNormal)
    {
        // Distance to plane
        float distanceToPlane = dot(triNormal, p - triPoint0);
        // calculate point q (projection of p on the triangle)
        vec3 q = p - (distanceToPlane * triNormal);

        // Calculate the area of q to each set of two points
        float q01Area = triangleSurface(q, triPoint0, triPoint1);
        float q02Area = triangleSurface(q, triPoint0, triPoint2);
        float q12Area = triangleSurface(q, triPoint1, triPoint2);

        // If the area of q to each set of two points is equal to the triangle surface area, q is on the triangle
        if (q01Area + q02Area + q12Area == triangleSurface(triPoint0, triPoint1, triPoint2))
        {
            // The distance is simply the distance between the original point and the projected point of p (q)
            return distance(p, q);
        }

        //Point q is not on the triangle, check distance toward each edge of the triangle
        float sqDistance01 = squaredDistanceToLine(p, triPoint0, triPoint1);
        float sqDistance02 = squaredDistanceToLine(p, triPoint0, triPoint2);
        float sqDistance12 = squaredDistanceToLine(p, triPoint1, triPoint2);

        // Assume the shortest distance is sqDistance01
        // Then check if this is true
        float shortestDistance = sqDistance01;
        if (sqDistance02 < sqDistance12)
        {
            if (sqDistance02 < sqDistance01) shortestDistance = sqDistance02;
        }
        else if (sqDistance12 < sqDistance01) shortestDistance = sqDistance12;

        return sqrt(shortestDistance);
    }

    /**@brief Calculates the distance between a point and a triangle plane
     * @brief Calculates the normal of the triangle plane and calls pointToTriangle with normal
     * @param p - The point
     * @param triPoint0 - The first triangle point
     * @param triPoint1 - The second triangle point
     * @param triPoint2 - The last triangle point
     */
    float pointToTriangle(const vec3& p, vec3 triPoint0, vec3 triPoint1, vec3 triPoint2)
    {
        vec3 normal = normalize(cross(triPoint1 - triPoint0, triPoint2 - triPoint0));
        return pointToTriangle(p, triPoint0, triPoint1, triPoint2, normal);
    }

    /**@class triangle
     * @brief Data structure for a triangle
     */
    struct triangle
    {
        triangle(vec3 p0, vec3 p1, vec3 p2)
        {
            points[0] = p0;
            points[1] = p1;
            points[2] = p2;
            normal = normalize(cross(p1 - p0, p2 - p0));
        }

        triangle(vec3 p0, vec3 p1, vec3 p2, vec3 normal) :
            normal(normalize(normal))
        {
            points[0] = p0;
            points[1] = p1;
            points[2] = p2;
        }

        // The three points of the triangle
        vec3 points[3];
        // The normalized normal of the triangle plane
        vec3 normal;

        /**@brief Calculates the closest distance between point p and this triangle
         */
        float distanceToPoint(const vec3& p) const
        {
            // Distance to plane
            float distanceToPlane = dot(normal, p - points[0]);
            // calculate point q (projection of p on the triangle)
            vec3 q = p - (distanceToPlane*normal);

            // Calculate the area of q to each set of two points
            float q01Area = triangleSurface(q, points[0], points[1]);
            float q02Area = triangleSurface(q, points[0], points[2]);
            float q12Area = triangleSurface(q, points[1], points[2]);

            // If the area of q to each set of two points is equal to the triangle surface area, q is on the triangle
            if (q01Area + q02Area + q12Area == surface())
            {
                // The distance is simply the distance between the original point and the projected point of p (q)
                return distance(p, q);
            }

            //Point q is not on the triangle, check distance toward each edge of the triangle
            float sqDistance01 = squaredDistanceToLine(p, points[0], points[1]);
            float sqDistance02 = squaredDistanceToLine(p, points[0], points[2]);
            float sqDistance12 = squaredDistanceToLine(p, points[1], points[2]);

            // Assume the shortest distance is sqDistance01
            // Then check if this is true
            float shortestDistance = sqDistance01;
            if (sqDistance02 < sqDistance12)
            {
                if (sqDistance02 < sqDistance01) shortestDistance = sqDistance02;
            }
            else if (sqDistance12 < sqDistance01) shortestDistance = sqDistance12;

            return sqrt(shortestDistance);
        }

        /**@brief Calculates the size of the surface area
         */
        float surface() const
        {
            return 0.5 * (normalizeDot(points[0], points[1]) * normalizeDot(points[0], points[2])) * fastSin(0);
        }
    };

    /**@brief Calculates the closest distance between point p and a plane
     * @param point - The point
     * @param planePosition - A point on the plane
     * @param planeNormal - The plane normal
     */
    float pointToPlane(vec3 point, vec3 planePosition, vec3 planeNormal)
    {
        return dot(planeNormal, point - planePosition);
    }

    /**@class plane
     * @brief 
     */
    struct plane
    {
        plane(vec3 position, vec3 normal) :
            position(position), normal(normalize(normal))
        {

        }

        /**@brief Constructs a plane from three points on the plane
         * @brief Uses p0 for the plane position 
         * @brief Calculates a normal using math::cross
         */
        plane(vec3 p0, vec3 p1, vec3 p2)
        {
            position = p0;
            normal = normalize(cross(p1 - p0, p2 - p0));
        }

        // Position on the plane, or a point on the plane
        vec3 position;
        // Normalized normal of the plane
        vec3 normal;

        /**@brief Calculates the closest distance between point p and this plane
         */
        float distanceToPoint(const vec3& p) const
        {
            return dot(normal, p-position);
        }
    };
}
