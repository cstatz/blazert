#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <blazert/blazert.h>
#include <blaze/math/Column.h>

#define NOMINMAX
#include "../common/tiny_obj_loader.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../common/stb_image_write.h"

using ft = double;

constexpr ft pi = 3.141592653589793238462643383279502884;
constexpr size_t uMaxBounces = 10;
constexpr int SPP = 100;

using namespace blazert;

template<typename T>
inline T uniform(T min, T max) {
  return min + T(std::rand()) / T(RAND_MAX) * (max - min);
}

// Building an Orthonormal Basis, Revisited: http://jcgt.org/published/0006/01/01/
template<typename T>
inline void revised_onb(const Vec3r<T> &n, Vec3r<T> &b1, Vec3r<T> &b2) {
  if (n[2] < static_cast<T>(0.0)) {
    const T a = static_cast<T>(1.0) / (static_cast<T>(1.0) - n[2]);
    const T b = n[0] * n[1] * a;
    b1 = {static_cast<T>(1.0) - n[0] * n[0] * a, -b, n[0]};
    b2 = {b, n[1] * n[1] * a - static_cast<T>(1.0), -n[1]};
  }
  else {
    const T a = static_cast<T>(1.0) / (static_cast<T>(1.0) + n[2]);
    const T b = -n[0] * n[1] * a;
    b1 = {static_cast<T>(1.0) - n[0] * n[0] * a, b, -n[0]};
    b2 = {b, static_cast<T>(1.0) - n[1] * n[1] * a, -n[1]};
  }
}

template<typename T>
inline Vec3r<T> direction_cos_theta(const Vec3r<T> &normal) {

  const T u1 = uniform(T(0.), T(1.));
  const T phi = uniform(T(0.), T(2.) * static_cast<T>(pi));

  const T r{std::sqrt(u1)};

  const T x{r * std::cos(phi)};
  const T y{r * std::sin(phi)};
  const T z{std::sqrt(static_cast<T>(1.0) - u1)};

  Vec3r<T> xDir;
  Vec3r<T> yDir;
  revised_onb(normal, xDir, yDir);

  return {xDir * x + yDir * y + z * normal};
}

template<typename T>
inline T pdf_a_to_w(const T a_pdf_a, const T a_dist, const T a_cos_there){
  return a_pdf_a * (a_dist * a_dist) / std::abs(a_cos_there);
}

template<typename T>
struct Mesh {
  Vec3rList<T> vertices;
  Vec3iList faces;/// [xyz] * num_vertices
  std::vector<unsigned int> material_ids;
  Mat3rList<T> facevarying_uvs;    /// [xyz] * 3(triangle) * num_faces -> this may be Mat2rList<T>
};

class EmissiveFace {
public:
  explicit EmissiveFace(unsigned int f = 0, unsigned int m = 0) : face_(f), mtl_(m) {}
  unsigned int face_;
  unsigned int mtl_;
};

template<typename T>
class MeshLight {
public:
  MeshLight(const Mesh<T> &mesh, const std::vector<tinyobj::material_t> &materials) : mesh_(mesh), materials_(materials) {
    for (unsigned int face = 0; face < mesh_.faces.size(); face++) {
      unsigned int mtl_id = mesh_.material_ids[face];
      const tinyobj::material_t &faceMtl = materials_[mtl_id];

      if (faceMtl.emission[0] > static_cast<T>(0.0) || faceMtl.emission[1] > static_cast<T>(0.0) || faceMtl.emission[2] > static_cast<T>(0.0)) {
        EmissiveFace ef(face, mtl_id);
        emissive_faces_.push_back(ef);
      }
    }
  }

  void sample_direct(const Vec3r<T> &x, T Xi1, T Xi2, Vec3r<T> &dstDir, T &dstDist, T &dstPdf, Vec3r<T> &dstRadiance) const {
    unsigned int num_faces = emissive_faces_.size();
    unsigned int face = std::min(static_cast<unsigned int>(std::floor(Xi1 * num_faces)), num_faces - 1);
    T lightPickPdf = static_cast<T>(1.0) / static_cast<T>(num_faces);

    // normalize random number
    Xi1 = Xi1 * num_faces - face;

    unsigned int fid = emissive_faces_[face].face_;
    unsigned int mtlid = emissive_faces_[face].mtl_;

    const Vec3ui &ff = mesh_.faces[fid];

    const Vec3r<T> &v0 = mesh_.vertices[ff[0]];
    const Vec3r<T> &v1 = mesh_.vertices[ff[1]];
    const Vec3r<T> &v2 = mesh_.vertices[ff[2]];

    const T Xi1_ = std::sqrt(Xi1);
    const T c0 = static_cast<T>(1.0) - Xi1_;
    const T c1 = Xi1_ * (static_cast<T>(1.0) - Xi2);
    const T c2 = Xi1_ * Xi2;

    Vec3r<T> tmp = cross(v1 - v0, v2 - v0);
    Vec3r<T> norm = normalize(tmp);
    T area = length(tmp) / static_cast<T>(2.0);

    dstPdf = static_cast<T>(0.0);
    const Vec3r<T> lp{c0 * v0 + c1 * v1 + c2 * v2};
    T areaPdf = lightPickPdf * (static_cast<T>(1.0) / area);
    dstDir = lp - x;
    dstDist = length(dstDir);

    Vec3r<T> ll = materials_[mtlid].emission;
    if (dstDist > static_cast<T>(0.000001)) {
      dstDir = normalize(dstDir);
      T cosAtLight = std::max(dot(-dstDir, norm), static_cast<T>(0.0));
      dstRadiance = ll * cosAtLight;// light has cosine edf

      // convert pdf to solid angle measure
      dstPdf = pdf_a_to_w(areaPdf, dstDist, cosAtLight);
    }
  }

  std::vector<EmissiveFace> emissive_faces_;
  const Mesh<T> &mesh_;
  const std::vector<tinyobj::material_t> &materials_;
};

// TODO: This eats float ... we need to convert.
void SaveImagePNG(const char *filename, const float *rgb, const unsigned int width,
                  const unsigned int height) {
  auto *bytes = new unsigned char[width * height * 3];
  for (unsigned int y = 0; y < height; y++) {
    for (unsigned int x = 0; x < width; x++) {
      const unsigned int index = y * width + x;
      bytes[index * 3 + 0] = (unsigned char) std::max(
          0.0f, std::min(rgb[index * 3 + 0] * 255.0f, 255.0f));
      bytes[index * 3 + 1] = (unsigned char) std::max(
          0.0f, std::min(rgb[index * 3 + 1] * 255.0f, 255.0f));
      bytes[index * 3 + 2] = (unsigned char) std::max(
          0.0f, std::min(rgb[index * 3 + 2] * 255.0f, 255.0f));
    }
  }
  stbi_write_png(filename, static_cast<int>(width), static_cast<int>(height), 3, bytes, static_cast<int>(width * 3));
  delete[] bytes;
}

template<typename T>
bool LoadObj(Mesh<T> &mesh, std::vector<tinyobj::material_t> &materials, const char *filename, float scale, const char *mtl_path) {

  std::vector<tinyobj::shape_t> shapes;
  std::string err = tinyobj::LoadObj(shapes, materials, filename, mtl_path);

  if (!err.empty()) {
    std::cerr << err << std::endl;
    return false;
  }

  size_t num_vertices = 0;
  size_t num_faces = 0;
  for (size_t i = 0; i < shapes.size(); i++) {
    num_vertices += shapes[i].mesh.positions.size() / 3;
    num_faces += shapes[i].mesh.indices.size() / 3;
  }

  size_t vertexIdxOffset = 0;

  for (size_t i = 0; i < shapes.size(); i++) {
    // Faces + Mats
    for (size_t f = 0; f < shapes[i].mesh.indices.size() / 3; f++) {
      mesh.faces.push_back(Vec3ui{shapes[i].mesh.indices[3 * f + 0], shapes[i].mesh.indices[3 * f + 1], shapes[i].mesh.indices[3 * f + 2]} + vertexIdxOffset);
      mesh.material_ids.push_back(static_cast<unsigned int>(shapes[i].mesh.material_ids[f]));
    }

    // Vertices
    for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
      mesh.vertices.push_back(scale * Vec3r<ft>{shapes[i].mesh.positions[3 * v + 0], shapes[i].mesh.positions[3 * v + 1], shapes[i].mesh.positions[3 * v + 2]});
    }

    vertexIdxOffset = mesh.vertices.size();

    if (shapes[i].mesh.texcoords.size() > 0) {
      for (size_t f = 0; f < shapes[i].mesh.indices.size() / 3; f++) {

        const unsigned int f0 = shapes[i].mesh.indices[3 * f + 0];
        const unsigned int f1 = shapes[i].mesh.indices[3 * f + 1];
        const unsigned int f2 = shapes[i].mesh.indices[3 * f + 2];

        const Vec3r<ft> n0{shapes[i].mesh.texcoords[2 * f0 + 0], shapes[i].mesh.texcoords[2 * f0 + 1], 0.};
        const Vec3r<ft> n1{shapes[i].mesh.texcoords[2 * f1 + 0], shapes[i].mesh.texcoords[2 * f1 + 1], 0.};
        const Vec3r<ft> n2{shapes[i].mesh.texcoords[2 * f2 + 0], shapes[i].mesh.texcoords[2 * f2 + 1], 0.};

        Mat3r<ft> n{};
        blaze::column<0UL>(n) = n0;
        blaze::column<1UL>(n) = n1;
        blaze::column<2UL>(n) = n2;
        mesh.facevarying_uvs.emplace_back(n);
      }
    }
  }

  return true;
}

template<typename T>
inline T sign(T f) { return f < 0 ? static_cast<T>(-1) : static_cast<T>(1); }

template<typename T>
inline Vec3r<T> reflect(const Vec3r<T> &I, const Vec3r<T> &N) { return {I - T(2.) * dot(I, N) * N}; }

template<typename T>
inline Vec3r<T> refract(const Vec3r<T> &I, const Vec3r<T> &N, T eta) {
  const T NdotI = dot(N, I);
  const T k = static_cast<T>(1.0) - eta * eta * (static_cast<T>(1.0) - NdotI * NdotI);
  if (k < static_cast<T>(0.0))
    return Vec3r<T>{0.};
  else
    return {eta * I - (eta * NdotI + std::sqrt(k)) * N};
}

template<typename T>
inline T pow5(const T val) { return {val * val * val * val * val}; }

template<typename T>
inline T fresnel_schlick(const Vec3r<T> &H, const Vec3r<T> &norm, T n1) {
  const T r0 = n1 * n1;
  return r0 + (static_cast<T>(1.) - r0) * pow5(static_cast<T>(1.) - dot(H, norm));
}

void progressBar(unsigned long tick, unsigned long total, unsigned long width = 100) {
  float ratio = 100.0f * tick / total;
  float count = width * tick / total;
  std::string bar(width, ' ');
  std::fill(bar.begin(), bar.begin() + count, '+');
  std::cout << "[ " << ratio << "%% ] [ " << bar << " ]" << (tick == total ? '\n' : '\r');
  std::fflush(stdout);
}

/// Check for occlusion
template<typename T>
inline bool check_for_occluder(const Vec3r<T> &p1, const Vec3r<T> &p2, const Mesh<T> &mesh, const blazert::BVH<T, TriangleMesh> &bvh) {

  static const T ray_eps = T(0.00001);

  Vec3r<T> dir{p2 - p1};
  const T dist = length(dir);

  const blazert::Ray<T> shadow_ray{p1, dir, ray_eps, dist - ray_eps, blazert::Ray<T>::CullBackFace::no, blazert::Ray<T>::AnyHit::yes};

  blazert::RayHit<T> rayhit;
  return traverse(bvh, shadow_ray, rayhit);
}

int main(int argc, char **argv) {

  unsigned int width = 512;
  unsigned int height = 512;

  ft scale{1.0};

  std::string objFilename = "models/cornellbox_suzanne_lucy.obj";
  std::string mtlPath = "models/";

  if (argc > 1) {
    objFilename = std::string(argv[1]);
  }

  if (argc > 2) {
    scale = ft(std::atof(argv[2]));
  }

  if (argc > 3) {
    mtlPath = std::string(argv[3]);
  }

#ifdef _OPENMP
  std::cout << "-> Using OpenMP!\n";
#endif

  auto *mesh = new Mesh<ft>;

  std::vector<tinyobj::material_t> materials;
  const bool ret = LoadObj(*mesh, materials, objFilename.c_str(), scale, mtlPath.c_str());
  if (!ret) {
    std::cerr << "Failed to load [ " << objFilename.c_str() << " ]\n";
    return -1;
  }

  MeshLight lights(*mesh, materials);

  blazert::BVHBuildOptions<ft> build_options;// Use default option

  blazert::TriangleMesh triangle_mesh(mesh->vertices, mesh->faces);
  blazert::BVH accel(triangle_mesh);
  blazert::SAHBinnedBuilder builder;
  [[maybe_unused]] auto build_statistics = builder.build(accel, build_options);


  std::vector<float> rgb(width * height * 3, 0.0f);

  std::srand(0);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
  for (unsigned int y = 0; y < height; y++) {
    for (unsigned int x = 0; x < width; x++) {
      Vec3r<ft> finalColor{0, 0, 0};
      for (int i = 0; i < SPP; ++i) {
        ft px = ft(x) + uniform(ft(-0.5), ft(0.5));
        ft py = ft(y) + uniform(ft(-0.5), ft(0.5));

        Vec3r<ft> rayOrg{0.0, 5.0, 20.0};

        Vec3r<ft> rayDir{(px / static_cast<ft>(width)) - static_cast<ft>(0.5),
                         (py / static_cast<ft>(height)) - static_cast<ft>(0.5),
                         static_cast<ft>(-1.0)};

        Vec3r<ft> color{0., 0., 0.};
        Vec3r<ft> weight{1., 1., 1.};

        bool do_emission = true;// just skip emission if light sampling was done on previous event (No MIS)
        for (size_t b = 0; b < uMaxBounces; ++b) {
          // Russian Roulette
          ft rr_fac{1.0};
          if (b > 3) {
            ft rr_rand = uniform(0., 1.);
            ft termination_probability{0.2};
            if (rr_rand < termination_probability) {
              break;
            }
            rr_fac = static_cast<ft>(1.0) - termination_probability;
          }
          weight *= (1.0 / rr_fac);

          const blazert::Ray<ft> ray{rayOrg, rayDir, ft(0.001)};

          blazert::RayHit<ft> rayhit;
          if (!traverse(accel, ray, rayhit))
            break;

          rayOrg += (ray.direction * rayhit.hit_distance);

          unsigned int fid = rayhit.prim_id;
          Vec3r<ft> norm = -rayhit.normal;

          // Flip normal torwards incoming ray for backface shading
          Vec3r<ft> originalNorm{norm};
          if (dot(norm, ray.direction) > static_cast<ft>(0.)) {
            norm *= static_cast<ft>(-1.);
          }

          // Get properties from the material of the hit primitive
          unsigned int matId = mesh->material_ids[fid];
          tinyobj::material_t mat = materials[matId];

          Vec3r<ft> diffuseColor(mat.diffuse);
          Vec3r<ft> emissiveColor(mat.emission);
          Vec3r<ft> specularColor(mat.specular);
          Vec3r<ft> refractionColor(mat.transmittance);
          ft ior = mat.ior;

          // Calculate fresnel factor based on ior.
          ft inside = sign(dot(ray.direction, originalNorm));// 1 for inside, -1 for outside
          // Assume ior of medium outside of objects = 1.0
          ft n1 = inside < 0 ? static_cast<ft>(1.0) / ior : ior;
          ft n2 = static_cast<ft>(1.0) / n1;

          ft fresnel = fresnel_schlick(Vec3r<ft>{-ray.direction}, norm, static_cast<ft>((n1 - n2) / (n1 + n2)));

          // Compute probabilities for each surface interaction.
          // Specular is just regular reflectiveness * fresnel.
          ft rhoS = dot(Vec3r<ft>{1, 1, 1} / static_cast<ft>(3.0), specularColor) * fresnel;
          // If we don't have a specular reflection, choose either diffuse or
          // transmissive
          // Mix them based on the dissolve value of the material
          ft rhoD = dot(Vec3r<ft>{1, 1, 1} / static_cast<ft>(3.0), diffuseColor) * (static_cast<ft>(1.0) - fresnel) * (static_cast<ft>(1.0) - mat.dissolve);
          ft rhoR = dot(Vec3r<ft>{1, 1, 1} / static_cast<ft>(3.0), refractionColor) * (static_cast<ft>(1.0) - fresnel) * mat.dissolve;
          ft rhoE = dot(Vec3r<ft>{1, 1, 1} / static_cast<ft>(3.0), emissiveColor);

          // Normalize probabilities so they sum to 1.0
          ft totalrho = rhoS + rhoD + rhoR + rhoE;
          // No scattering event is likely, just stop here
          if (totalrho < static_cast<ft>(0.0001)) {
            break;
          }

          rhoS /= totalrho;
          rhoD /= totalrho;
          rhoR /= totalrho;

          // Choose an interaction based on the calculated probabilities
          ft rand = uniform(0., 1.);
          Vec3r<ft> outDir;
          // REFLECT glossy
          if (rand < rhoS) {
            outDir = reflect(ray.direction, norm);
            weight *= specularColor;
            do_emission = true;
          }
          // REFLECT diffuse
          else if (rand < (rhoS + rhoD)) {
            Vec3r<ft> brdfEval{static_cast<ft>(1.0) / pi * diffuseColor};
            Vec3r<ft> ldir, ll;
            ft lpdf, ldist;
            lights.sample_direct(rayOrg, uniform(0., 1.), uniform(0., 1.), ldir, ldist, lpdf, ll);

            if (lpdf > static_cast<ft>(0.0)) {
              ft cosTheta = std::abs(dot(ldir, norm));
              Vec3r<ft> directLight{brdfEval * ll * cosTheta / lpdf};
              bool visible = !check_for_occluder(rayOrg, Vec3r<ft>{rayOrg + ldir * ldist}, *mesh, accel);

              Vec3r<ft> temp_color = directLight * weight;
              temp_color *=  static_cast<ft>(visible);
              color += temp_color;
            }

            // Sample cosine weighted hemisphere
            outDir = direction_cos_theta(norm);
            weight *= diffuseColor;
            do_emission = false;
          }
          // REFRACT
          else if (rand < (rhoD + rhoS + rhoR)) {
            outDir = refract(ray.direction, Vec3r<ft>{-inside * originalNorm}, n1);
            weight *= refractionColor;
            do_emission = true;
          }
          // EMIT
          else {
            if (do_emission) {
              color += std::max(dot(originalNorm, -ray.direction), static_cast<ft>(0.0)) * (emissiveColor * weight);
            }
            break;
          }
          // Calculate new ray origin and set outgoing direction.
          rayDir = outDir;
        }

        finalColor += color;
      }

      finalColor *= 1.0 / SPP;

      // Correct gamma
      finalColor[0] = pow(finalColor[0], 1.0 / 2.2);
      finalColor[1] = pow(finalColor[1], 1.0 / 2.2);
      finalColor[2] = pow(finalColor[2], 1.0 / 2.2);

      rgb[3 * ((height - y - 1) * width + x) + 0] = float(finalColor[0]);
      rgb[3 * ((height - y - 1) * width + x) + 1] = float(finalColor[1]);
      rgb[3 * ((height - y - 1) * width + x) + 2] = float(finalColor[2]);
    }

    progressBar(y + 1, height);
  }

  SaveImagePNG("render.png", &rgb.at(0), width, height);

  return 0;
}
