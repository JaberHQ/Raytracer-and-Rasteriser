#pragma once

//constexpr float epsilon = 1e-3f;
constexpr float epsilon = std::numeric_limits<float>::epsilon();

glm::vec3 DoNothing(triangle* tri, int depth, glm::vec3 p, glm::vec3 dir)
{
    return vec3(0);
}

/* Compute the colour of a triangle */
glm::vec3 Shade( triangle* tri, int depth, glm::vec3 p, glm::vec3 dir )
{
    // Colour with small ambient amount
    glm::vec3 colour = tri->v1.col * 0.1f;

    // Surface normal - Normalised unit vector 
    glm::vec3 n = glm::normalize( tri->v1.nor );

    // Light direction - Normalised unit vector 
    glm::vec3 l = glm::normalize( light_pos - p );

    // Light distance
    float l_distance = glm::length( light_pos - p );

    /* Shadows */
    float t = FLT_MAX;
    glm::vec3 shadow_colour;
    trace( p + n * 0.001f, l, t, shadow_colour, depth, DoNothing );

    // If shadow ray does not hit or hits background
    if( t >= l_distance || shadow_colour == bkgd )
    {
        // Diffuse - Lamberts law
        float diffuse = glm::max<float>( glm::dot( n, l ), 0.0f );
        colour += tri->v1.col * diffuse;
    }

    /* Reflection */
    if( tri->reflect && depth <= max_recursion_depth )
    {
        glm::vec3 reflect_dir = glm::reflect( dir, n );
        glm::vec3 reflect_colour;
        trace( p + n * 0.001f, reflect_dir, t, reflect_colour, depth + 1, Shade );
        colour += reflect_colour * 0.5f;
    }

    return colour;
}

/* Returns if the point is within the triangle */
bool PointInTriangle( glm::vec3 pt, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, float t)
{
    ///* Calculate if first vertex is inside a triangle whose three points lie on the plane */
    //glm::vec3 v1q = pt - v1;
    //glm::vec3 v12 = v2 - v1;
    //glm::vec3 v13 = v3 - v1;
    //float d1 = glm::dot( glm::cross( v12, v1q ), glm::cross( v12, v13 ) );

    ///* Calculate if second vertex is inside a triangle whose three points lie on the plane */
    //glm::vec3 v23 = v3 - v2;
    //glm::vec3 v2q = pt - v2;
    //glm::vec3 v21 = v1 - v2;
    //float d2 = glm::dot( glm::cross( v23, v2q ), glm::cross( v23, v21 ) );

    ///* Calculate if third vertex is inside a triangle whose three points lie on the plane */
    //glm::vec3 v31 = v1 - v3;
    //glm::vec3 v3q = pt - v3;
    //glm::vec3 v32 = v2 - v3;
    //float d3 = glm::dot( glm::cross( v31, v3q ), glm::cross( v31, v32 ) );
    //
    ///* Determine if the point q is inside the triangle */
    //if( d1 > 0 && d2 > 0 && d3 > 0 )
    //{
    //    return true;
    //}
    //return false;


   if (t > epsilon)
        return true;
    return false;

}

/* Test the intersection of a ray with a triangle */
float RayTriangleIntersection(glm::vec3 o, glm::vec3 dir, triangle* tri, glm::vec3& point)
{
    ///* Vertices of triangle */
    //glm::vec3 v1 = tri->v1.pos;
    //glm::vec3 v2 = tri->v2.pos;
    //glm::vec3 v3 = tri->v3.pos;
    //
    ///* Edge */
    //glm::vec3 v12 = v2 - v1;
    //glm::vec3 v13 = v3 - v1;

    //glm::vec3 N = glm::cross( v12, v13 ); //  Normal of the plane

    ///* If ray is parallel to the plane */
    //float NRay = glm::dot( dir, N );
    //if( fabs( NRay ) < epsilon )
    //    return FLT_MAX; // No intersection

    //float t = glm::dot( v1 - o, N ) / glm::dot( dir, N ); // Distance from ray origin to intersection point

    //if( t < 0.0f )
    //    return FLT_MAX; // No intersection

    //glm::vec3 q = o + ( dir * t ); // The point q which the ray intersects the plane

    ///* If intersection point is inside triangle */
    //if( PointInTriangle( q, v1, v2, v3 ) )
    //{
    //    point = q; // Store intersection point
    //    return t;
    //}
    //return FLT_MAX; // Intersection point is outside the triangle


    glm::vec3 v1 = tri->v1.pos;
    glm::vec3 v2 = tri->v2.pos;
    glm::vec3 v3 = tri->v3.pos;
    
    glm::vec3 edge1 = v2 - v1;
    glm::vec3 edge2 = v3 - v1;
    glm::vec3 ray_cross_e2 = glm::cross( dir, edge2 );
    float det = glm::dot( edge1, ray_cross_e2 );
    
    if( det > -epsilon && det < epsilon )
        return FLT_MAX;
    
    float inv_det = 1.0f / det;
    glm::vec3 s = o - v1;
    float u = inv_det * dot( s, ray_cross_e2 );
    
    if( ( u < 0 && std::abs( u ) > epsilon ) || ( u > 1 && std::abs( u - 1 ) > epsilon ) )
    return FLT_MAX;
    
    glm::vec3 s_cross_e1 = glm::cross( s, edge1 );
    float v = inv_det * glm::dot( dir, s_cross_e1 );
    
    if( ( v < 0 && std::abs( v ) > epsilon ) || ( u + v > 1 && std::abs( u + v - 1 ) > epsilon ) )
    return FLT_MAX;
    
    float t = inv_det * glm::dot( edge2, s_cross_e1 );
    
    if( PointInTriangle( point, v1, v2, v3, t ) )
    {
        point = o + dir * t;
        return t;
    }
    return FLT_MAX;
}

/* Find the closest intersection of a ray, compute the colour at intersection
   and handle shading */
void trace(glm::vec3 o, glm::vec3 dir, float& t, glm::vec3& io_col, int depth, closest_hit p_hit)
{
    float closest = FLT_MAX;
    triangle* closest_triangle = nullptr;
    glm::vec3 closest_point;

    /* For each tri in triangles */
    for( int i = 0; i < tris.size(); i++ )
    {
        glm::vec3 point;
        float t_temp = RayTriangleIntersection( o, dir, &tris[ i ], point ); // Calculate t

        /* Save closest triangle */
        if( t_temp <  closest && t_temp > epsilon)
        {
            closest = t_temp;
            closest_triangle = &tris[ i ];
            closest_point = point;
        }
    }

    t = closest;
    if( closest_triangle )
    {
        // Compute colour
        if( p_hit )
        {
            io_col = p_hit( closest_triangle, depth, closest_point, dir );
        }
        else
        {
            io_col = closest_triangle->v1.col; // Use colour of the closest triangle
        }
    }
    else
    {
        io_col = bkgd; // Set to background colour
    }
}

/* Calculate ray direction */
vec3 GetRayDirection(float px, float py, int W, int H, float aspect_ratio, float fov)
{
    float a = aspect_ratio; // Aspect ratio of image
    float f = glm::tan( glm::radians( fov ) * 0.5f ); // Vertical fov

    /* Camera coords */
    glm::vec3 R = glm::vec3( 1.0f, 0.0f, 0.0f ); // Right
    glm::vec3 U = glm::vec3( 0.0f, -1.0f, 0.0f ); // Up
    glm::vec3 F = glm::vec3( 0.0f, 0.0f, -1.0f ); // Forward

    /* Pixel coords to normalised device coords*/
    float x = ( ( 2 * ( px + 0.5f ) ) / W ) - 1; // x pixel coord
    float y = ( ( 2 * ( py + 0.5f ) ) / H ) - 1; // y pixel coord
    glm::vec3 d = aspect_ratio * f * x * R + f * y * U + F; // Direction vector

    glm::vec3 nomralise_d = glm::normalize( d );
    return nomralise_d;
}

/* Whitted Ray Tracer */
void raytrace()
{
    // For each pixel_y in PIXEL_H
    for (int pixel_y = 0; pixel_y < PIXEL_H; ++pixel_y)
    {
        float percf = (float)pixel_y / (float)PIXEL_H;
        int perci = percf * 100;
        std::clog << "\rScanlines done: " << perci << "%" << ' ' << std::flush;

        // For each pixel_x in PIXEL_W
        for (int pixel_x = 0; pixel_x < PIXEL_W; ++pixel_x)
        {
            /* Calculate ray */
            const float aspect_ratio = (float)PIXEL_W / (float)PIXEL_H;
            const float fov = 90.0f;
            glm::vec3 ray = GetRayDirection( (float)pixel_x, (float)pixel_y, (float)PIXEL_W, (float)PIXEL_H, aspect_ratio, fov );
            
            /* Calcualte the colour of pixel */
            float t;
            glm::vec3 pixel_colour;
            trace( eye, ray, t, pixel_colour, max_recursion_depth, Shade );
            writeCol( pixel_colour, pixel_x, pixel_y );
        }
    }
    std::clog << "\rFinish rendering.           \n";
}
