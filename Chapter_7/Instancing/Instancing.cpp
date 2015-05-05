// The MIT License (MIT)
//
// Copyright (c) 2013 Dan Ginsburg, Budirijanto Purnomo
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

//
// Book:      OpenGL(R) ES 3.0 Programming Guide, 2nd Edition
// Authors:   Dan Ginsburg, Budirijanto Purnomo, Dave Shreiner, Aaftab Munshi
// ISBN-10:   0-321-93388-5
// ISBN-13:   978-0-321-93388-1
// Publisher: Addison-Wesley Professional
// URLs:      http://www.opengles-book.com
//            http://my.safaribooksonline.com/book/animation-and-3d/9780133440133
//
// Instancing.c
//
//    Demonstrates drawing multiple objects in a single draw call with
//    geometry instancing
//
#include <stdlib.h>
#include <math.h>
#include "esUtil.h"


#ifdef _WIN32
#define srandom srand
#define random rand
#endif


#define NUM_INSTANCES   100
#define POSITION_LOC    0
#define COLOR_LOC       1
#define MVP_LOC         2

#include "../../Math/Vector.h"
#include "../../Math/Matrix.h"
#include "../../Math/Quaternion.h"

using namespace Mathematics;

class Cube
{
public:
    
    /// Physics state.
    
    struct State
    {
        /// primary physics state
        
        Vector position;                ///< the position of the cube center of mass in world coordinates (meters).
        Vector momentum;                ///< the momentum of the cube in kilogram meters per second.
        Quaternion orientation;         ///< the orientation of the cube represented by a unit quaternion.
        Vector angularMomentum;         ///< angular momentum vector.
        
        // secondary state
        
        Vector velocity;                ///< velocity in meters per second (calculated from momentum).
        Quaternion spin;                ///< quaternion rate of change in orientation.
        Vector angularVelocity;         ///< angular velocity (calculated from angularMomentum).
        Matrix bodyToWorld;             ///< body to world coordinates matrix.
        Matrix worldToBody;             ///< world to body coordinates matrix.
        
        /// constant state
        
        float size;                     ///< length of the cube sides in meters.
        float mass;                     ///< mass of the cube in kilograms.
        float inverseMass;              ///< inverse of the mass used to convert momentum to velocity.
        float inertiaTensor;            ///< inertia tensor of the cube (i have simplified it to a single value due to the mass properties a cube).
        float inverseInertiaTensor;     ///< inverse inertia tensor used to convert angular momentum to angular velocity.
        
        /// Recalculate secondary state values from primary values.
        
        void recalculate()
        {
            velocity = momentum * inverseMass;
            angularVelocity = angularMomentum * inverseInertiaTensor;
            orientation.normalize();
            spin = 0.5 * Quaternion(0, angularVelocity.x, angularVelocity.y, angularVelocity.z) * orientation;
            Matrix translation;
            translation.translate(position);
            bodyToWorld = translation * orientation.matrix();
            worldToBody = bodyToWorld.inverse();
        }
    };
    
    /// Default constructor.
    
    Cube()
    {
        current.size = 1;
        current.mass = 1;
        current.inverseMass = 1.0f / current.mass;
        current.position = Vector(2,0,0);
        current.momentum = Vector(0,0,-10);
        current.orientation.identity();
        current.angularMomentum = Vector(0,0,0);
        current.inertiaTensor = current.mass * current.size * current.size * 1.0f / 6.0f;
        current.inverseInertiaTensor = 1.0f / current.inertiaTensor;
        current.recalculate();
        previous = current;
    }
    
    /// Update physics state.
    
    void update(float t, float dt)
    {
        previous = current;
        integrate(current, t, dt);
    }
    
    /// Render cube at interpolated state.
    /// Calculates interpolated state then renders cube at the interpolated
    /// position and orientation using OpenGL.
    /// @param alpha interpolation alpha in [0,1]
    
    void render(float alpha = 1.0f)
    {
        
    }
    
private:
    
    /// Interpolate between two physics states.
    
    static State interpolate(const State &a, const State &b, float alpha)
    {
        State state = b;
        state.position = a.position*(1-alpha) + b.position*alpha;
        state.momentum = a.momentum*(1-alpha) + b.momentum*alpha;
        state.orientation = slerp(a.orientation, b.orientation, alpha);
        state.angularMomentum = a.angularMomentum*(1-alpha) + b.angularMomentum*alpha;
        state.recalculate();
        return state;
    }
    
    State previous;     ///< previous physics state.
    State current;      ///< current physics state.
    
    /// Derivative values for primary state.
    /// This structure stores all derivative values for primary state in Cube::State.
    /// For example velocity is the derivative of position, force is the derivative
    /// of momentum etc. Storing all derivatives in this structure makes it easy
    /// to implement the RK4 integrator cleanly because it needs to calculate the
    /// and store derivative values at several points each timestep.
    
    struct Derivative
    {
        Vector velocity;                ///< velocity is the derivative of position.
        Vector force;                   ///< force in the derivative of momentum.
        Quaternion spin;                ///< spin is the derivative of the orientation quaternion.
        Vector torque;                  ///< torque is the derivative of angular momentum.
    };
    
    /// Evaluate all derivative values for the physics state at time t.
    /// @param state the physics state of the cube.
    
    static Derivative evaluate(const State &state, float t)
    {
        Derivative output;
        output.velocity = state.velocity;
        output.spin = state.spin;
        forces(state, t, output.force, output.torque);
        return output;
    }
    
    /// Evaluate derivative values for the physics state at future time t+dt
    /// using the specified set of derivatives to advance dt seconds from the
    /// specified physics state.
    
    static Derivative evaluate(State state, float t, float dt, const Derivative &derivative)
    {
        state.position += derivative.velocity * dt;
        state.momentum += derivative.force * dt;
        state.orientation += derivative.spin * dt;
        state.angularMomentum += derivative.torque * dt;
        state.recalculate();
        
        Derivative output;
        output.velocity = state.velocity;
        output.spin = state.spin;
        forces(state, t+dt, output.force, output.torque);
        return output;
    }
    
    /// Integrate physics state forward by dt seconds.
    /// Uses an RK4 integrator to numerically integrate with error O(5).
    
    static void integrate(State &state, float t, float dt)
    {
        Derivative a = evaluate(state, t);
        Derivative b = evaluate(state, t, dt*0.5f, a);
        Derivative c = evaluate(state, t, dt*0.5f, b);
        Derivative d = evaluate(state, t, dt, c);
        
        state.position += 1.0f/6.0f * dt * (a.velocity + 2.0f*(b.velocity + c.velocity) + d.velocity);
        state.momentum += 1.0f/6.0f * dt * (a.force + 2.0f*(b.force + c.force) + d.force);
        state.orientation += 1.0f/6.0f * dt * (a.spin + 2.0f*(b.spin + c.spin) + d.spin);
        state.angularMomentum += 1.0f/6.0f * dt * (a.torque + 2.0f*(b.torque + c.torque) + d.torque);
        
        state.recalculate();
    }
    
    /// Calculate force and torque for physics state at time t.
    /// Due to the way that the RK4 integrator works we need to calculate
    /// force implicitly from state rather than explictly applying forces
    /// to the rigid body once per update. This is because the RK4 achieves
    /// its accuracy by detecting curvature in derivative values over the
    /// timestep so we need our force values to supply the curvature.
    
    static void forces(const State &state, float t, Vector &force, Vector &torque)
    {
        // attract towards origin
        
        force = -10 * state.position;
        
        // sine force to add some randomness to the motion
        
        force.x += 10 * sin(t*0.9f + 0.5f);
        force.y += 11 * sin(t*0.5f + 0.4f);
        force.z += 12 * sin(t*0.7f + 0.9f);
        
        // sine torque to get some spinning action
        
        torque.x = 1.0f * sin(t*0.9f + 0.5f);
        torque.y = 1.1f * sin(t*0.5f + 0.4f);
        torque.z = 1.2f * sin(t*0.7f + 0.9f);
        
        // damping torque so we dont spin too fast
        
        torque -= 0.2f * state.angularVelocity;
    }
};

typedef struct
{
   // Handle to a program object
   GLuint programObject;

   // VBOs
   GLuint positionVBO;
   GLuint colorVBO;
   GLuint mvpVBO;
   GLuint indicesIBO;

   // Number of indices
   int       numIndices;

   // Rotation angle
   GLfloat   angle[NUM_INSTANCES];

} UserData;

///
// Initialize the shader and program object
//
int Init ( ESContext *esContext )
{
   GLfloat *positions;
   GLuint *indices;

   UserData *userData = (UserData *) esContext->userData;
   const char vShaderStr[] =
      "#version 300 es                             \n"
      "layout(location = 0) in vec4 a_position;    \n"
      "layout(location = 1) in vec4 a_color;       \n"
      "layout(location = 2) in mat4 a_mvpMatrix;   \n"
      "out vec4 v_color;                           \n"
      "void main()                                 \n"
      "{                                           \n"
      "   v_color = a_color;                       \n"
      "   gl_Position = a_mvpMatrix * a_position;  \n"
      "}                                           \n";

   const char fShaderStr[] =
      "#version 300 es                                \n"
      "precision mediump float;                       \n"
      "in vec4 v_color;                               \n"
      "layout(location = 0) out vec4 outColor;        \n"
      "void main()                                    \n"
      "{                                              \n"
      "  outColor = v_color;                          \n"
      "}                                              \n";

   // Load the shaders and get a linked program object
   userData->programObject = esLoadProgram ( vShaderStr, fShaderStr );

   // Generate the vertex data
   userData->numIndices = esGenCube ( 0.1f, &positions,
                                      NULL, NULL, &indices );

   // Index buffer object
   glGenBuffers ( 1, &userData->indicesIBO );
   glBindBuffer ( GL_ELEMENT_ARRAY_BUFFER, userData->indicesIBO );
   glBufferData ( GL_ELEMENT_ARRAY_BUFFER, sizeof ( GLuint ) * userData->numIndices, indices, GL_STATIC_DRAW );
   glBindBuffer ( GL_ELEMENT_ARRAY_BUFFER, 0 );
   free ( indices );

   // Position VBO for cube model
   glGenBuffers ( 1, &userData->positionVBO );
   glBindBuffer ( GL_ARRAY_BUFFER, userData->positionVBO );
   glBufferData ( GL_ARRAY_BUFFER, 24 * sizeof ( GLfloat ) * 3, positions, GL_STATIC_DRAW );
   free ( positions );

   // Random color for each instance
   {
      GLubyte colors[NUM_INSTANCES][4];
      int instance;

      srandom ( 0 );

      for ( instance = 0; instance < NUM_INSTANCES; instance++ )
      {
         colors[instance][0] = random() % 255;
         colors[instance][1] = random() % 255;
         colors[instance][2] = random() % 255;
         colors[instance][3] = 0;
      }

      glGenBuffers ( 1, &userData->colorVBO );
      glBindBuffer ( GL_ARRAY_BUFFER, userData->colorVBO );
      glBufferData ( GL_ARRAY_BUFFER, NUM_INSTANCES * 4, colors, GL_STATIC_DRAW );
   }

   // Allocate storage to store MVP per instance
   {
      int instance;

      // Random angle for each instance, compute the MVP later
      for ( instance = 0; instance < NUM_INSTANCES; instance++ )
      {
         userData->angle[instance] = ( float ) ( random() % 32768 ) / 32767.0f * 360.0f;
      }

      glGenBuffers ( 1, &userData->mvpVBO );
      glBindBuffer ( GL_ARRAY_BUFFER, userData->mvpVBO );
      glBufferData ( GL_ARRAY_BUFFER, NUM_INSTANCES * sizeof ( ESMatrix ), NULL, GL_DYNAMIC_DRAW );
   }
   glBindBuffer ( GL_ARRAY_BUFFER, 0 );

   glClearColor ( 1.0f, 1.0f, 1.0f, 0.0f );
   return GL_TRUE;
}


///
// Update MVP matrix based on time
//
void Update ( ESContext *esContext, float deltaTime )
{
    /*
   UserData *userData = ( UserData * ) esContext->userData;
   ESMatrix *matrixBuf;
   ESMatrix perspective;
   float    aspect;
   int      instance = 0;
   int      numRows;
   int      numColumns;


   // Compute the window aspect ratio
   aspect = ( GLfloat ) esContext->width / ( GLfloat ) esContext->height;

   // Generate a perspective matrix with a 60 degree FOV
   esMatrixLoadIdentity ( &perspective );
   esPerspective ( &perspective, 60.0f, aspect, 1.0f, 20.0f );

   glBindBuffer ( GL_ARRAY_BUFFER, userData->mvpVBO );
   matrixBuf = ( ESMatrix * ) glMapBufferRange ( GL_ARRAY_BUFFER, 0, sizeof ( ESMatrix ) * NUM_INSTANCES, GL_MAP_WRITE_BIT );

   // Compute a per-instance MVP that translates and rotates each instance differnetly
   numRows = ( int ) sqrtf ( NUM_INSTANCES );
   numColumns = numRows;

   for ( instance = 0; instance < NUM_INSTANCES; instance++ )
   {
      ESMatrix modelview;
      float translateX = ( ( float ) ( instance % numRows ) / ( float ) numRows ) * 2.0f - 1.0f;
      float translateY = ( ( float ) ( instance / numColumns ) / ( float ) numColumns ) * 2.0f - 1.0f;

      // Generate a model view matrix to rotate/translate the cube
      esMatrixLoadIdentity ( &modelview );

      // Per-instance translation
      esTranslate ( &modelview, translateX, translateY, -2.0f );

      // Compute a rotation angle based on time to rotate the cube
      userData->angle[instance] += ( deltaTime * 40.0f );

      if ( userData->angle[instance] >= 360.0f )
      {
         userData->angle[instance] -= 360.0f;
      }

      // Rotate the cube
//      esRotate ( &modelview, userData->angle[instance], 1.0, 0.0, 1.0 );

      // Compute the final MVP by multiplying the
      // modevleiw and perspective matrices together
      esMatrixMultiply ( &matrixBuf[instance], &modelview, &perspective );
   }

   glUnmapBuffer ( GL_ARRAY_BUFFER );
     */
}

///
// Draw a triangle using the shader pair created in Init()
//
void Draw ( ESContext *esContext )
{
   UserData *userData = (UserData *)esContext->userData;

   // Set the viewport
   glViewport ( 0, 0, esContext->width, esContext->height );

   // Clear the color buffer
   glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

   // Use the program object
   glUseProgram ( userData->programObject );

   // Load the vertex position
   glBindBuffer ( GL_ARRAY_BUFFER, userData->positionVBO );
   glVertexAttribPointer ( POSITION_LOC, 3, GL_FLOAT,
                           GL_FALSE, 3 * sizeof ( GLfloat ), ( const void * ) NULL );
   glEnableVertexAttribArray ( POSITION_LOC );

   // Load the instance color buffer
   glBindBuffer ( GL_ARRAY_BUFFER, userData->colorVBO );
   glVertexAttribPointer ( COLOR_LOC, 4, GL_UNSIGNED_BYTE,
                           GL_TRUE, 4 * sizeof ( GLubyte ), ( const void * ) NULL );
   glEnableVertexAttribArray ( COLOR_LOC );
   glVertexAttribDivisor ( COLOR_LOC, 1 ); // One color per instance


   // Load the instance MVP buffer
   glBindBuffer ( GL_ARRAY_BUFFER, userData->mvpVBO );

   // Load each matrix row of the MVP.  Each row gets an increasing attribute location.
   glVertexAttribPointer ( MVP_LOC + 0, 4, GL_FLOAT, GL_FALSE, sizeof ( ESMatrix ), ( const void * ) NULL );
   glVertexAttribPointer ( MVP_LOC + 1, 4, GL_FLOAT, GL_FALSE, sizeof ( ESMatrix ), ( const void * ) ( sizeof ( GLfloat ) * 4 ) );
   glVertexAttribPointer ( MVP_LOC + 2, 4, GL_FLOAT, GL_FALSE, sizeof ( ESMatrix ), ( const void * ) ( sizeof ( GLfloat ) * 8 ) );
   glVertexAttribPointer ( MVP_LOC + 3, 4, GL_FLOAT, GL_FALSE, sizeof ( ESMatrix ), ( const void * ) ( sizeof ( GLfloat ) * 12 ) );
   glEnableVertexAttribArray ( MVP_LOC + 0 );
   glEnableVertexAttribArray ( MVP_LOC + 1 );
   glEnableVertexAttribArray ( MVP_LOC + 2 );
   glEnableVertexAttribArray ( MVP_LOC + 3 );

   // One MVP per instance
   glVertexAttribDivisor ( MVP_LOC + 0, 1 );
   glVertexAttribDivisor ( MVP_LOC + 1, 1 );
   glVertexAttribDivisor ( MVP_LOC + 2, 1 );
   glVertexAttribDivisor ( MVP_LOC + 3, 1 );

   // Bind the index buffer
   glBindBuffer ( GL_ELEMENT_ARRAY_BUFFER, userData->indicesIBO );

   // Draw the cubes
   glDrawElementsInstanced ( GL_TRIANGLES, userData->numIndices, GL_UNSIGNED_INT, ( const void * ) NULL, NUM_INSTANCES );
}

///
// Cleanup
//
void Shutdown ( ESContext *esContext )
{
   UserData *userData = (UserData *)esContext->userData;

   glDeleteBuffers ( 1, &userData->positionVBO );
   glDeleteBuffers ( 1, &userData->colorVBO );
   glDeleteBuffers ( 1, &userData->mvpVBO );
   glDeleteBuffers ( 1, &userData->indicesIBO );

   // Delete program object
   glDeleteProgram ( userData->programObject );
}

#ifdef __cplusplus
extern "C" {
#endif
    int esMain ( ESContext *esContext )
    {
        esContext->userData = malloc ( sizeof ( UserData ) );
        
        esCreateWindow ( esContext, "Instancing", 640, 480, ES_WINDOW_RGB | ES_WINDOW_DEPTH );
        
        if ( !Init ( esContext ) )
        {
            return GL_FALSE;
        }
        
        esRegisterShutdownFunc ( esContext, Shutdown );
        esRegisterUpdateFunc ( esContext, Update );
        esRegisterDrawFunc ( esContext, Draw );
        
        return GL_TRUE;
    }
#ifdef __cplusplus
}
#endif
