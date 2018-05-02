/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MESH_FILTER_H
#define CNOID_UTIL_MESH_FILTER_H

#include "exportdecl.h"

namespace cnoid {

class SgNode;
class SgMesh;
class MeshFilterImpl;

class CNOID_EXPORT MeshFilter
{
public:
    MeshFilter();
    MeshFilter(const MeshFilter& org);
    ~MeshFilter();

    void setNormalOverwritingEnabled(bool on);
    void setMinCreaseAngle(float angle);
    void setMaxCreaseAngle(float angle);
    bool generateNormals(SgMesh* mesh, float creaseAngle = 3.14159f, bool removeRedundantVertices = false);

    void removeRedundantVertices(SgNode* node);
    void removeRedundantVertices(SgMesh* mesh);
    
    void removeRedundantFaces(SgNode* node);
    void removeRedundantFaces(SgMesh* mesh);

    // Deprecated. Use enableNormalOverwriting()
    void setOverwritingEnabled(bool on);

private:
    MeshFilterImpl* impl;
};

// for the backward compatibility
typedef MeshFilter MeshNormalGenerator;

}

#endif
