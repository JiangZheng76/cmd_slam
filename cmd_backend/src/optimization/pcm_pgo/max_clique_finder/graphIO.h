#pragma once

#include <float.h>
#include <string.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#define LINE_LENGTH 256

using namespace std;

namespace FMC {
typedef std::vector<int> IntVector;

class CGraphIO {
 public:
  CGraphIO() {}
  virtual ~CGraphIO();

  bool readGraph(string s_InputFile, float connStrength = -DBL_MAX);
  string getFileExtension(string fileName);
  bool ReadMatrixMarketAdjacencyGraph(string s_InputFile,
                                      float connStrength = -DBL_MAX);
  bool ReadEigenAdjacencyMatrix(const Eigen::MatrixXd& adjMatrix);
  bool ReadMeTiSAdjacencyGraph(string s_InputFile);
  void CalculateVertexDegrees();

  int GetVertexCount() { return m_vi_Vertices.size() - 1; }
  int GetEdgeCount() { return m_vi_Edges.size() / 2; }
  int GetMaximumVertexDegree() { return m_i_MaximumVertexDegree; }
  int GetMinimumVertexDegree() { return m_i_MinimumVertexDegree; }
  double GetAverageVertexDegree() { return m_d_AverageVertexDegree; }
  string GetInputFile() { return m_s_InputFile; }

  vector<int>* GetVerticesPtr() { return &m_vi_Vertices; }
  vector<int>* GetEdgesPtr() { return &m_vi_Edges; }

 public:
  int m_i_MaximumVertexDegree;
  int m_i_MinimumVertexDegree;
  double m_d_AverageVertexDegree;

  string m_s_InputFile;

  vector<int> m_vi_Vertices;
  vector<int> m_vi_OrderedVertices;
  vector<int> m_vi_Edges;
  vector<double> m_vd_Values;
};
}  // namespace FMC
