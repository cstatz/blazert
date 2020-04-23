
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cctype>

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>

#include "tiny_obj_loader.h"

namespace tinyobj {

#define TINYOBJ_SSCANF_BUFFER_SIZE  (4096)

struct vertex_index {
  int v_idx, vt_idx, vn_idx;
  vertex_index(){};
  vertex_index(int idx) : v_idx(idx), vt_idx(idx), vn_idx(idx){};
  vertex_index(int vidx, int vtidx, int vnidx)
      : v_idx(vidx), vt_idx(vtidx), vn_idx(vnidx){};
};
// for std::map
static inline bool operator<(const vertex_index &a, const vertex_index &b) {
  if (a.v_idx != b.v_idx)
    return (a.v_idx < b.v_idx);
  if (a.vn_idx != b.vn_idx)
    return (a.vn_idx < b.vn_idx);
  if (a.vt_idx != b.vt_idx)
    return (a.vt_idx < b.vt_idx);

  return false;
}

struct obj_shape {
  std::vector<double> v;
  std::vector<double> vn;
  std::vector<double> vt;
};

static inline bool isSpace(const char c) { return (c == ' ') || (c == '\t'); }

static inline bool isNewLine(const char c) {
  return (c == '\r') || (c == '\n') || (c == '\0');
}

// Make index zero-base, and also support relative index.
static inline int fixIndex(int idx, int n) {
  if (idx > 0) return idx - 1;
  if (idx == 0) return 0;
  return n + idx; // negative value = relative
}

static inline std::string parseString(const char *&token) {
  std::string s;
  token += strspn(token, " \t");
  size_t e = strcspn(token, " \t\r");
  s = std::string(token, &token[e]);
  token += e;
  return s;
}

static inline int parseInt(const char *&token) {
  token += strspn(token, " \t");
  int i = atoi(token);
  token += strcspn(token, " \t\r");
  return i;
}


// Tries to parse a doubleing point number located at s.
//
// s_end should be a location in the string where reading should absolutely
// stop. For example at the end of the string, to prevent buffer overflows.
//
// Parses the following EBNF grammar:
//   sign    = "+" | "-" ;
//   END     = ? anything not in digit ?
//   digit   = "0" | "1" | "2" | "3" | "4" | "5" | "6" | "7" | "8" | "9" ;
//   integer = [sign] , digit , {digit} ;
//   decimal = integer , ["." , integer] ;
//   double   = ( decimal , END ) | ( decimal , ("E" | "e") , integer , END ) ;
//
//  Valid strings are for example:
//   -0	 +3.1417e+2  -0.0E-3  1.0324  -1.41   11e2
//
// If the parsing is a success, result is set to the parsed value and true 
// is returned.
//
// The function is greedy and will parse until any of the following happens:
//  - a non-conforming character is encountered.
//  - s_end is reached.
//
// The following situations triggers a failure:
//  - s >= s_end.
//  - parse failure.
// 
static bool tryParseDouble(const char *s, const char *s_end, double *result)
{
	if (s >= s_end)
	{
		return false;
	}

	double mantissa = 0.0;
	// This exponent is base 2 rather than 10.
	// However the exponent we parse is supposed to be one of ten,
	// thus we must take care to convert the exponent/and or the 
	// mantissa to a * 2^E, where a is the mantissa and E is the
	// exponent.
	// To get the final double we will use ldexp, it requires the
	// exponent to be in base 2.
	int exponent = 0;

	// NOTE: THESE MUST BE DECLARED HERE SINCE WE ARE NOT ALLOWED
	// TO JUMP OVER DEFINITIONS.
	char sign = '+';
	char exp_sign = '+';
	char const *curr = s;

	// How many characters were read in a loop. 
	int read = 0;
	// Tells whether a loop terminated due to reaching s_end.
	bool end_not_reached = false;

	/*
		BEGIN PARSING.
	*/

	// Find out what sign we've got.
	if (*curr == '+' || *curr == '-')
	{
		sign = *curr;
		curr++;
	}
	else if (isdigit(*curr)) { /* Pass through. */ }
	else
	{
		goto fail;
	}

	// Read the integer part.
	while ((end_not_reached = (curr != s_end)) && isdigit(*curr))
	{
		mantissa *= 10;
		mantissa += static_cast<int>(*curr - 0x30);
		curr++;	read++;
	}

	// We must make sure we actually got something.
	if (read == 0)
		goto fail;
	// We allow numbers of form "#", "###" etc.
	if (!end_not_reached)
		goto assemble;

	// Read the decimal part.
	if (*curr == '.')
	{
		curr++;
		read = 1;
		while ((end_not_reached = (curr != s_end)) && isdigit(*curr))
		{
			// NOTE: Don't use powf here, it will absolutely murder precision.
			mantissa += static_cast<int>(*curr - 0x30) * pow(10.0, -read);
			read++; curr++;
		}
	}
	else if (*curr == 'e' || *curr == 'E') {}
	else
	{
		goto assemble;
	}

	if (!end_not_reached)
		goto assemble;

	// Read the exponent part.
	if (*curr == 'e' || *curr == 'E')
	{
		curr++;
		// Figure out if a sign is present and if it is.
		if ((end_not_reached = (curr != s_end)) && (*curr == '+' || *curr == '-'))
		{
			exp_sign = *curr;
			curr++;
		}
		else if (isdigit(*curr)) { /* Pass through. */ }
		else
		{
			// Empty E is not allowed.
			goto fail;
		}

		read = 0;
		while ((end_not_reached = (curr != s_end)) && isdigit(*curr))
		{
			exponent *= 10;
			exponent += static_cast<int>(*curr - 0x30);
			curr++;	read++;
		}
		exponent *= (exp_sign == '+'? 1 : -1);
		if (read == 0)
			goto fail;
	}

assemble:
	*result = (sign == '+'? 1 : -1) * ldexp(mantissa * pow(5.0, exponent), exponent);
	return true;
fail:
	return false;
}
static inline double parseFloat(const char *&token) {
  token += strspn(token, " \t");
#ifdef TINY_OBJ_LOADER_OLD_FLOAT_PARSER
  double f = (double)atof(token);
  token += strcspn(token, " \t\r");
#else
  const char *end = token + strcspn(token, " \t\r");
  double val = 0.0;
  tryParseDouble(token, end, &val);
  double f = static_cast<double>(val);
  token = end;
#endif
  return f;
}


static inline void parseFloat2(double &x, double &y, const char *&token) {
  x = parseFloat(token);
  y = parseFloat(token);
}

static inline void parseFloat3(double &x, double &y, double &z,
                               const char *&token) {
  x = parseFloat(token);
  y = parseFloat(token);
  z = parseFloat(token);
}

// Parse triples: i, i/j/k, i//k, i/j
static vertex_index parseTriple(const char *&token, int vsize, int vnsize,
                                int vtsize) {
  vertex_index vi(-1);

  vi.v_idx = fixIndex(atoi(token), vsize);
  token += strcspn(token, "/ \t\r");
  if (token[0] != '/') {
    return vi;
  }
  token++;

  // i//k
  if (token[0] == '/') {
    token++;
    vi.vn_idx = fixIndex(atoi(token), vnsize);
    token += strcspn(token, "/ \t\r");
    return vi;
  }

  // i/j/k or i/j
  vi.vt_idx = fixIndex(atoi(token), vtsize);
  token += strcspn(token, "/ \t\r");
  if (token[0] != '/') {
    return vi;
  }

  // i/j/k
  token++; // skip '/'
  vi.vn_idx = fixIndex(atoi(token), vnsize);
  token += strcspn(token, "/ \t\r");
  return vi;
}

static unsigned int
updateVertex(std::map<vertex_index, unsigned int> &vertexCache,
             std::vector<double> &positions, std::vector<double> &normals,
             std::vector<double> &texcoords,
             const std::vector<double> &in_positions,
             const std::vector<double> &in_normals,
             const std::vector<double> &in_texcoords, const vertex_index &i) {
  const std::map<vertex_index, unsigned int>::iterator it = vertexCache.find(i);

  if (it != vertexCache.end()) {
    // found cache
    return it->second;
  }

  assert(in_positions.size() > (unsigned int)(3 * i.v_idx + 2));

  positions.push_back(in_positions[3 * i.v_idx + 0]);
  positions.push_back(in_positions[3 * i.v_idx + 1]);
  positions.push_back(in_positions[3 * i.v_idx + 2]);

  if (i.vn_idx >= 0) {
    normals.push_back(in_normals[3 * i.vn_idx + 0]);
    normals.push_back(in_normals[3 * i.vn_idx + 1]);
    normals.push_back(in_normals[3 * i.vn_idx + 2]);
  }

  if (i.vt_idx >= 0) {
    texcoords.push_back(in_texcoords[2 * i.vt_idx + 0]);
    texcoords.push_back(in_texcoords[2 * i.vt_idx + 1]);
  }

  unsigned int idx = static_cast<unsigned int>(positions.size() / 3 - 1);
  vertexCache[i] = idx;

  return idx;
}


static bool exportFaceGroupToShape(
    shape_t &shape, std::map<vertex_index, unsigned int> vertexCache,
    const std::vector<double> &in_positions,
    const std::vector<double> &in_normals,
    const std::vector<double> &in_texcoords,
    const std::vector<std::vector<vertex_index> > &faceGroup,
    const int material_id, const std::string &name, bool clearCache) {
  if (faceGroup.empty()) {
    return false;
  }

  // Flatten vertices and indices
  for (size_t i = 0; i < faceGroup.size(); i++) {
    const std::vector<vertex_index> &face = faceGroup[i];

    vertex_index i0 = face[0];
    vertex_index i1(-1);
    vertex_index i2 = face[1];

    size_t npolys = face.size();

    // Polygon -> triangle fan conversion
    for (size_t k = 2; k < npolys; k++) {
      i1 = i2;
      i2 = face[k];

      unsigned int v0 = updateVertex(
          vertexCache, shape.mesh.positions, shape.mesh.normals,
          shape.mesh.texcoords, in_positions, in_normals, in_texcoords, i0);
      unsigned int v1 = updateVertex(
          vertexCache, shape.mesh.positions, shape.mesh.normals,
          shape.mesh.texcoords, in_positions, in_normals, in_texcoords, i1);
      unsigned int v2 = updateVertex(
          vertexCache, shape.mesh.positions, shape.mesh.normals,
          shape.mesh.texcoords, in_positions, in_normals, in_texcoords, i2);

      shape.mesh.indices.push_back(v0);
      shape.mesh.indices.push_back(v1);
      shape.mesh.indices.push_back(v2);

    }
  }

  shape.name = name;

  if (clearCache)
    vertexCache.clear();

  return true;
}

std::string LoadObj(std::vector<shape_t> &shapes,
                    std::istream &inStream) {
  std::stringstream err;

  std::vector<double> v;
  std::vector<double> vn;
  std::vector<double> vt;
  std::vector<std::vector<vertex_index> > faceGroup;
  std::string name;

  std::map<vertex_index, unsigned int> vertexCache;

  shape_t shape;

  int maxchars = 8192;             // Alloc enough size.
  std::vector<char> buf(maxchars); // Alloc enough size.
  while (inStream.peek() != -1) {
    inStream.getline(&buf[0], maxchars);

    std::string linebuf(&buf[0]);

    // Trim newline '\r\n' or '\n'
    if (linebuf.size() > 0) {
      if (linebuf[linebuf.size() - 1] == '\n')
        linebuf.erase(linebuf.size() - 1);
    }
    if (linebuf.size() > 0) {
      if (linebuf[linebuf.size() - 1] == '\r')
        linebuf.erase(linebuf.size() - 1);
    }

    // Skip if empty line.
    if (linebuf.empty()) {
      continue;
    }

    // Skip leading space.
    const char *token = linebuf.c_str();
    token += strspn(token, " \t");

    assert(token);
    if (token[0] == '\0')
      continue; // empty line

    if (token[0] == '#')
      continue; // comment line

    // vertex
    if (token[0] == 'v' && isSpace((token[1]))) {
      token += 2;
      double x, y, z;
      parseFloat3(x, y, z, token);
      v.push_back(x);
      v.push_back(y);
      v.push_back(z);
      continue;
    }

    // normal
    if (token[0] == 'v' && token[1] == 'n' && isSpace((token[2]))) {
      token += 3;
      double x, y, z;
      parseFloat3(x, y, z, token);
      vn.push_back(x);
      vn.push_back(y);
      vn.push_back(z);
      continue;
    }

    // texcoord
    if (token[0] == 'v' && token[1] == 't' && isSpace((token[2]))) {
      token += 3;
      double x, y;
      parseFloat2(x, y, token);
      vt.push_back(x);
      vt.push_back(y);
      continue;
    }

    // face
    if (token[0] == 'f' && isSpace((token[1]))) {
      token += 2;
      token += strspn(token, " \t");

      std::vector<vertex_index> face;
      while (!isNewLine(token[0])) {
        vertex_index vi =
            parseTriple(token, static_cast<int>(v.size() / 3), static_cast<int>(vn.size() / 3), static_cast<int>(vt.size() / 2));
        face.push_back(vi);
        size_t n = strspn(token, " \t\r");
        token += n;
      }

      faceGroup.push_back(face);

      continue;
    }

    // group name
    if (token[0] == 'g' && isSpace((token[1]))) {

      // flush previous face group.
      bool ret = exportFaceGroupToShape(shape, vertexCache, v, vn, vt,
                                        faceGroup, 0, name, true);
      if (ret) {
        shapes.push_back(shape);
      }

      shape = shape_t();

      // material = -1;
      faceGroup.clear();

      std::vector<std::string> names;
      while (!isNewLine(token[0])) {
        std::string str = parseString(token);
        names.push_back(str);
        token += strspn(token, " \t\r"); // skip tag
      }

      assert(names.size() > 0);

      // names[0] must be 'g', so skip the 0th element.
      if (names.size() > 1) {
        name = names[1];
      } else {
        name = "";
      }

      continue;
    }

    // object name
    if (token[0] == 'o' && isSpace((token[1]))) {

      // flush previous face group.
      bool ret = exportFaceGroupToShape(shape, vertexCache, v, vn, vt,
                                        faceGroup, 0, name, true);
      if (ret) {
        shapes.push_back(shape);
      }

      // material = -1;
      faceGroup.clear();
      shape = shape_t();

      // @todo { multiple object name? }
      char namebuf[TINYOBJ_SSCANF_BUFFER_SIZE];
      token += 2;
#ifdef _MSC_VER
      sscanf_s(token, "%s", namebuf, (unsigned)_countof(namebuf));
#else
      sscanf(token, "%s", namebuf);
#endif
      name = std::string(namebuf);

      continue;
    }

    // Ignore unknown command.
  }

  bool ret = exportFaceGroupToShape(shape, vertexCache, v, vn, vt, faceGroup,
                                    0, name, true);
  if (ret) {
    shapes.push_back(shape);
  }
  faceGroup.clear(); // for safety

  return err.str();
}

std::string LoadObj(std::vector<shape_t> &shapes,
                    const char *filename) {

  shapes.clear();

  std::stringstream err;

  std::ifstream ifs(filename);
  if (!ifs) {
    err << "Cannot open file [" << filename << "]" << std::endl;
    return err.str();
  }

  return LoadObj(shapes, ifs);
}
}
