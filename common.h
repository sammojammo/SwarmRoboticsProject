#ifndef COMMON_H_
#define COMMON_H_

#include <list>
#include <vector>
using namespace std;

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "simobject.h"
#include "arguments.h"

typedef struct {
    double x;
    double y;
} TVector2d;

extern double GetDistanceBetweenPositions(const TVector2d* pt_pos1, const TVector2d* pt_pos2);
extern double GetSquaredDistanceBetweenPositions(const TVector2d* pt_pos1, const TVector2d* pt_pos2);

// Returns the squared length of a vector
#define Vec2dLengthSquared(vec) (vec.x*vec.x + vec.y*vec.y)

// Returns the length of a vector
#define Vec2dLength(vec) sqrt(Vec2dLengthSquared(vec))


// Multiply a vector by a scalar
#define Vec2dMultiplyScalar(vec, scalar) \
  {                                      \
      vec.x *= scalar;                   \
      vec.y *= scalar;                   \
  }                            

#define Vec2dSub(result, A, B)   \
 {                               \
     result.x = B.x - A.x;       \
     result.y = B.y - A.y;       \
 }
   
// Adds two vectors:
#define Vec2dAdd(result, A, B)   \
 {                               \
     result.x = B.x + A.x;       \
     result.y = B.y + A.y;       \
 }

// Find the cos to the angle between two vectors
#define Vec2dCosAngle(vec1, vec2) \
     ((vec1.x * vec2.x + vec1.y * vec2.y) / (Vec2dLength(vec1) * Vec2dLength(vec2))) 

// Find the angle between two vectors
#define Vec2dAngle(vec1, vec2) \
     (acos(Vec2dCosAngle(vec1, vec2)))

// Find the angle of one vector
#define Vec2dOwnAngle(vec) \
     (atan2(vec.y, vec.x))

#define Vec2dNormalize(vec)                             \
  {                                                     \
     double length___ = Vec2dLength(vec);                \
     vec.x /= length___;                                \
     vec.y /= length___;                                \
  }


// Returns the normalized angle in the range [0,2*M_PI)
#define NormalizeAngle(ang) \
  (ang < 0.0 ? fmod(ang, 2.0*M_PI)+2.0*M_PI : fmod(ang, 2.0*M_PI))

// Returns the normalized angle in the range [-PI,PI)
#define NormalizeAngleNegativePIPositivePI(ang) \
  (NormalizeAngle(ang) > M_PI ? NormalizeAngle(ang) - 2.0*M_PI : NormalizeAngle(ang))

// Rotate a vector:
#define Vec2dRotate(angle, vec)                         \
  {                                                     \
     double xt_ = vec.x;                                 \
     vec.x = cos(angle) * vec.x - sin(angle) * vec.y;   \
     vec.y = cos(angle) * vec.y + sin(angle) * xt_;     \
  }



#define PI 3.14159265
#define EPSILON 1e-10

#define ERRENDL fprintf(stderr, "\n");
#define PRINTPOS(label, vec) printf("%s, x: %2.6f, y: %2.6f\n", label, vec.x, vec.y);  
#define PRINTVEC2(label, vec) printf("%s, x: %2.6f, y: %2.6f\n", label, vec.x, vec.y);  
#define FILEANDLINE                                          { fprintf(stderr, "In %s:%d: ", __FILE__, __LINE__); }
#define ERROR(s)                                             { FILEANDLINE; fprintf(stderr, s); ERRENDL; }
#define ERROR1(s, p1)                                        { FILEANDLINE; fprintf(stderr, s, p1); ERRENDL; }
#define ERROR2(s, p1, p2)                                    { FILEANDLINE; fprintf(stderr, s, p1, p2); ERRENDL; }
#define ERROR3(s, p1, p2, p3)                                { FILEANDLINE; fprintf(stderr, s, p1, p2, p3); ERRENDL; }
#define ERROR4(s, p1, p2, p3, p4)                            { FILEANDLINE; fprintf(stderr, s, p1, p2, p3, p4 ); ERRENDL; }
#define ERROR5(s, p1, p2, p3, p4, p5)                        { FILEANDLINE; fprintf(stderr, s, p1, p2, p3, p4, p5 ); ERRENDL; }
#define ERROR6(s, p1, p2, p3, p4, p5, p6)                    { FILEANDLINE; fprintf(stderr, s, p1, p2, p3, p4, p5, p6 ); ERRENDL; }
#define ERROR7(s, p1, p2, p3, p4, p5, p6, p7 )               { FILEANDLINE; fprintf(stderr, s, p1, p2, p3, p4, p5, p6, p7 ); ERRENDL; }
#define ERROR8(s, p1, p2, p3, p4, p5, p6, p7, p8 )           { FILEANDLINE; fprintf(stderr, s, p1, p2, p3, p4, p5, p6, p7, p8 ); ERRENDL; }
#define ERROR9(s, p1, p2, p3, p4, p5, p6, p7, p8, p9 )       { FILEANDLINE; fprintf(stderr, s, p1, p2, p3, p4, p5, p6, p7, p8, p9 ); ERRENDL; }
#define ERROR10(s, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10 ) { FILEANDLINE; fprintf(stderr, s, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10 ); ERRENDL; }

// If debugging is defined then:
#ifdef  DEBUG

#include <stdio.h>

#define DEBUGOUT(s)                  { fprintf(stderr, s); }
#define DEBUGOUT1(s, p1)             { fprintf(stderr, s, p1); }
#define DEBUGOUT2(s, p1, p2)         { fprintf(stderr, s, p1, p2); }
#define DEBUGOUT3(s, p1, p2, p3)     { fprintf(stderr, s, p1, p2, p3); }
#define DEBUGOUT4(s, p1, p2, p3, p4) { fprintf(stderr, s, p1, p2, p3, p4 ); }
#else
// Otherwise simply define the macros as being empty:
#define DEBUGOUT(s)                  
#define DEBUGOUT1(s, p1)             
#define DEBUGOUT2(s, p1, p2)         
#define DEBUGOUT3(s, p1, p2, p3)     
#define DEBUGOUT4(s, p1, p2, p3, p4) 
#endif


#endif
