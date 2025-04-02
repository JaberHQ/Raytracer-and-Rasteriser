#pragma once
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

/* Clear the colour buffer */
void ClearColourBuffer(float col[4])
{
	for( int i = 0; i < PIXEL_H; i++ )
	{
		for( int j = 0; j < PIXEL_W; j++ )
		{
			glm::vec3 colour( col[ 0 ], col[ 1 ], col[ 2 ] );
			writeColToDisplayBuffer( colour, j, i );
		}
	}
}

/* Clear the depth buffer */
void ClearDepthBuffer()
{
	for( int i = 0; i < PIXEL_H; i++ )
	{
		for( int j = 0; j < PIXEL_W; j++ )
		{
			depth_buffer[ i * PIXEL_W + j ] = 1.0f;
		}
	}
}

/* Transform the position of each triangle by multiplying each vertex with the transformation matrix */
void ApplyTransformationMatrix(glm::mat4 T, vector<triangle>& tris)
{
	for( int i = 0; i < tris.size(); i++ )
	{
		tris[ i ].v1.pos = T * tris[ i ].v1.pos;
		tris[ i ].v2.pos = T * tris[ i ].v2.pos;
		tris[ i ].v3.pos = T * tris[ i ].v3.pos;
	}
}

/* Apply perspective division */
void ApplyPerspectiveDivision(vector<triangle>& tris)
{
	for( int i = 0; i < tris.size(); i++ )
	{
		tris[ i ].v1.pos /= tris[ i ].v1.pos.w;
		tris[ i ].v2.pos /= tris[ i ].v2.pos.w;
		tris[ i ].v3.pos /= tris[ i ].v3.pos.w;

	}
}

/* Transform each triangles x and y coordinates from normalised device coordinates to screen space */
void ApplyViewportTransformation( int w, int h, vector<triangle>& tris )
{
	for( int i = 0; i < tris.size(); i++ )
	{
		tris[ i ].v1.pos.x = ( tris[ i ].v1.pos.x + 1.0f ) * 0.5f * w;
		tris[ i ].v2.pos.x = ( tris[ i ].v2.pos.x + 1.0f ) * 0.5f * w;
		tris[ i ].v3.pos.x = ( tris[ i ].v3.pos.x + 1.0f ) * 0.5f * w;

		tris[ i ].v1.pos.y = ( 1.0f - tris[ i ].v1.pos.y ) * 0.5f * h;
		tris[ i ].v2.pos.y = ( 1.0f - tris[ i ].v2.pos.y ) * 0.5f * h;
		tris[ i ].v3.pos.y = ( 1.0f - tris[ i ].v3.pos.y ) * 0.5f * h;
	}
}

/* Compute the barycentric coordinates of the triangle */
void ComputeBarycentricCoordinates(int px, int py, triangle t, float& alpha, float& beta, float& gamma)
{
	// || B - A || * || C - A ||* sin(0) 
	// ---------------------------------
	//				2

	// 2D 
	glm::vec2 v0( t.v1.pos.x, t.v1.pos.y );
	glm::vec2 v1( t.v2.pos.x, t.v2.pos.y );
	glm::vec2 v2( t.v3.pos.x, t.v3.pos.y );

	// Area of triangle
	float area = 0.5f * ( -v1.y * v2.x + v0.y * ( -v1.x + v2.x ) + v0.x * ( v1.y - v2.y ) + v1.x * v2.y );

	// Area of triangle p, v1, v2
	float area0 = 0.5f * ( v1.x * v2.y - v1.y * v2.x + ( v1.y - v2.y ) * px + ( v2.x - v1.x ) * py );
	
	// Area of triangle p, v2, v0
	float area1 = 0.5f * ( v2.x * v0.y - v2.y * v0.x + ( v2.y - v0.y ) * px + ( v0.x - v2.x ) * py );

	// Area of triangle p, v0, v1
	float area2 = 0.5f * ( v0.x * v1.y - v0.y * v1.x + ( v0.y - v1.y ) * px + ( v1.x - v0.x ) * py );

	// Calculate barycentic coords
	alpha = area0 / area;
	beta = area1 / area;
	gamma = area2 / area;
}

/* Calculate buffer values using barycentric coordinates */
void ShadeFragment(triangle tri, float& alpha, float& beta, float& gamma, glm::vec3& col, float& depth)
{
	// Colour Buffer
	col = alpha * tri.v1.col + beta * tri.v2.col + gamma * tri.v3.col;

	// Depth Buffer
	depth = alpha * tri.v1.pos.z + beta * tri.v2.pos.z + gamma * tri.v3.pos.z;
}

/* Rasterise triangles */
void Rasterise( vector<triangle>& tris )
{
	std::vector<float> depthBuffer( PIXEL_W * PIXEL_H, std::numeric_limits<float>::max() );

	// For each pixel y in Y dimension
	for( int py = 0; py < PIXEL_H; py++ )
	{
		// Scanline percentage
		float percf = (float)py / (float)PIXEL_H;
		int perci = percf * 100;
		std::clog << "\rScanlines done: " << perci << "%" << ' ' << std::flush;

		// For each pixel x in X dimension
		for( int px = 0; px < PIXEL_W; px++ )
		{
			float closestDepth = FLT_MAX;
			glm::vec3 finalColour( 0.0f );
			bool pixelHit = false;

			// For each triangle 
			for( int i = 0; i < tris.size(); i++ )
			{
				float alpha, beta, gamma;
				ComputeBarycentricCoordinates( px, py, tris[ i ], alpha, beta, gamma);

				// If pixel x, y is inside triangle t
				if( alpha >= 0 && beta >= 0 && gamma >= 0 )
				{
					// Calculate the fragment colour
					glm::vec3 colour;
					float depth;
					ShadeFragment( tris[ i ], alpha, beta, gamma, colour, depth);

					// Depth test
					int bufferIndex = py * PIXEL_W + px;
					if( depth < depthBuffer[ bufferIndex ] )
					{
						depthBuffer[ bufferIndex ] = depth;
						finalColour = colour;
						pixelHit = true;
					}
				}
			}

			// If hit, update display buffer
			if( pixelHit )
				writeColToDisplayBuffer( finalColour, px, py );
		}
	}
	std::clog << "\rFinish rendering.           \n";
}

/* Initialise transformation matrix default values */
glm::mat4 TransformationMatrixInit(glm::vec3 model_translation, glm::vec3 cameraPos, float fovy, float zNear, float zFar )
{
	glm::mat4 model = glm::mat4( 1.0f );
	model = glm::translate( model, model_translation );

	glm::mat4 view = glm::mat4( 1.0f );
	//glm::vec3 cameraPos = glm::vec3( 0.0f, 0.0f, 0.1f );
	glm::vec3 cameraTarget = glm::vec3( 0.0f, 0.0f, 0.0f );
	glm::vec3 cameraUp = glm::vec3( 0.0f, 1.0f, 0.0f );
	view = glm::lookAt( cameraPos, cameraTarget, cameraUp );

	glm::mat4 projection = glm::mat4( 1.0f );
	projection = glm::perspective( glm::radians( fovy ), (float)PIXEL_W / (float)PIXEL_H, zNear, zFar );
	glm::mat4 mvpMatrix = projection * view * model;

	return mvpMatrix;
}

/* The defaults for the cornell box obj file */
glm::mat4 CornerllBoxInit()
{
	glm::vec3 model_translation = glm::vec3( 0.1f, -2.5f, -6.0f );
	glm::vec3 cameraPos = glm::vec3( 0.0f, 0.0f, 3.0f );
	glm::mat4 mvpMatrix = TransformationMatrixInit( model_translation, cameraPos, 60.0f, 0.1f, 10.0f );

	return mvpMatrix;
}

/* The defaults for the quad obj file */
glm::mat4 QuadInit()
{
	glm::vec3 model_translation = glm::vec3( 0.0f, 0.0f, -1.0f );
	glm::vec3 cameraPos = glm::vec3( 0.0f, 0.0f, 0.1f );
	glm::mat4 mvpMatrix = TransformationMatrixInit( model_translation, cameraPos, 60.0f, 0.1f, 10.0f );

	return mvpMatrix;
}

/* Render loop */
void render(vector<triangle>& tris)
{
	GLfloat bgd[ 4 ] = { 1.0f, 1.0f, 1.0f, 1.0f };
	ClearColourBuffer( bgd );
	ClearDepthBuffer();

	/* Use the correct transformation matrix based on which obj you're trying to render */ 
	ApplyTransformationMatrix( CornerllBoxInit(), tris ); // Ideally i'd make MODEL_PATH accessible and use that to do this automatically but oh well
	//ApplyTransformationMatrix( QuadInit(), tris );

	ApplyPerspectiveDivision( tris );
	ApplyViewportTransformation( PIXEL_W, PIXEL_H, tris );
	Rasterise( tris );
}
	
