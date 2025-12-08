#include <cstring>
#include <cmath>
#include <numbers>
#include <ai.h>
#include <ai_nodes.h>
#include <ai_allocate.h>

#include "misc/vec.h"

// core computation
vec3f tonemap(const vec3f& c) {
    auto x = max(vec3f(0.0f), vec3f(c) - vec3f(0.004f));
    return (x * (6.2f * x + 0.5f)) / (x * (6.2f * x + 1.7f) + 0.06f);
}

float erf(float x) {
    auto P = std::sqrt(std::numbers::pi_v<float>) / 2.0f;
    auto e = std::exp(-x*x);
    // Bürmann series
    return copysign(1.0f, x) / P * std::sqrt(1.0f - e) * (P + 31.0f/200.0f * e - 341.0f/8000.0f * e * e);
}

inline float coth(float x) {
    return (std::exp(-x) + std::exp(x)) / (std::exp(x) - std::exp(-x));
}

inline float sinh(float x) {
    return -0.5f * 1.0f / std::exp(x) + 0.5f * std::exp(x);
}

// cross section for Beckmann NDF with roughness m and dir cosine u
float sigma_beckmann_expanded(float u, float m) {
    if (0.0f == m) return (u + std::abs(u)) / 2.0f;
    float m2 = m*m;
    if (1.0f == u) return 1.0f - 0.5f * m2;
    float expansion_term = -0.25f * m2 * (u + std::abs(u));
    float u2 = u * u;
    return ((std::exp(u2 / (m2 * (-1.0f + u2))) * m * std::sqrt(1.0f - u2)) /
        std::sqrt(std::numbers::pi_v<float>) + u * (1.0f + erf(u / (m * std::sqrt(1.0f - u2))))) / 2.0f
        + expansion_term;
}

// vmf sigma
float sigma_vmf(float u, float m) {
    if (m < 0.25f) return sigma_beckmann_expanded(u, m);

    float m2 = m * m;
    float m4 = m2 * m2;
    float m8 = m4 * m4;

    float u2 = u * u;
    float u4 = u2 * u2;
    float u6 = u4 * u2;
    float u8 = u4 * u4;
    float u10 = u6 * u4;
    float u12 = u6 * u6;

    float coth2m2 = coth(2.0f / m2);
    float sinh2m2 = sinh(2.0f / m2);
    if (m > 0.9f) {
        return 0.25f - 0.25f * u * (m2 - 2.0f * coth2m2) + 0.0390625f *
            (-1.f + 3.f * u2) * (4.f + 3.f * m4 - 6.f * m2 * coth2m2);
    }

    return 0.25 - 0.25*u*(m2 - 2.*coth2m2) + 0.0390625*(-1. + 3.*u2)*(4. + 3.*m4 - 
        6.*m2*coth2m2) - 0.000732421875*(3. - 30.*u2 + 35.*u4)*(16. + 180.*m4 + 105.*m8 - 
        10.*m2*(8. + 21.*m4)*coth2m2) + 0.000049591064453125*(-5. + 105.*u2 - 315.*u4 + 231.*u6)*
        (64. + 105.*m4*(32. + 180.*m4 + 99.*m8) - 42.*m2*(16. + 240.*m4 + 495.*m8)*coth2m2) + 
        (1.0132789611816406e-6*(35. - 1260.*u2 + 6930.*u4 - 12012.*u6 + 6435.*u8)*(1. + coth2m2)*
        (-256. - 315.*m4*(128. + 33.*m4*(80. + 364.*m4 + 195.*m8)) + 18.*m2*(256. + 385.*m4*
        (32. + 312.*m4 + 585.*m8))*coth2m2)*sinh2m2)/exp(2./m2) - (9.12696123123169e-8*(-63. + 3465.*u2 
        - 30030.*u4 + 90090.*u6 - 109395.*u8 + 46189.*u10)*(1. + coth2m2)*(-1024. - 
        495.*m4*(768. + 91.*m4*(448. + 15.*m4*(448. + 1836.*m4 + 969.*m8))) + 110.*m2*(256. + 117.*m4*
        (256. + 21.*m4*(336. + 85.*m4*(32. + 57.*m4))))*coth2m2)*sinh2m2)/exp(2./m2)
        + (4.3655745685100555e-9*(231. - 18018.*u2 + 225225.*u4 - 1.02102e6*u6 + 2.078505e6*u8 
        - 1.939938e6*u10 + 676039.*u12)*(1. + coth2m2)*(-4096. - 3003.*m4*(1024. + 
        45.*m4*(2560. + 51.*m4*(1792. + 285.*m4*(80. + 308.*m4 + 161.*m8)))) + 78.*m2*(2048. + 385.*m4*
        (1280. + 153.*m4*(512. + 57.*m4*(192. + 35.*m4*(40. + 69.*m4)))))*coth2m2)*sinh2m2)/exp(2./m2);
}

vec3f erf(const vec3f& c) {
    return vec3(erf(c.x), erf(c.y), erf(c.z));
}

vec3f non_negtive(vec3f c) {
    return vec3f(max(c.x, 0.0f), max(c.y, 0.0f), max(c.z, 0.0f));
}

vec3f fm(float ui, float uo, float r, vec3f c) {
    vec3f C = sqrt(1.0 - c);
    vec3f Ck = (1.0 - 0.5441615108674713*C - 0.45302863761693374*(1.0 - c))/(1.0 + 1.4293127703064865*C);
    vec3f Ca = c / pow(1.0075 + 1.16942*C,atan((0.0225272 + (-0.264641 + r)*r)*Erf(C)));
    return non_negative(0.384016*(-0.341969 + Ca)*Ca*Ck*(-0.0578978/(0.287663 + ui*uo) + abs(-0.0898863 + tanh(r))));
}

vec3f vMF_diffuse(float ui, float uo, float phi, float r, vec3f c) {
    if (0.0f == r) return c / std::numbers::pi_v<float>;

    float m = -std::log(1.0 - std::sqrt(r));
    float sigmai = sigma_vmf(ui, m);
    float sigmao = sigma_vmf(uo, m);
    float sigmano = sigma_vmf(-uo, m);
    float sigio = sigmai * sigmao;
    float sigdenom = uo * sigmai + ui * sigmano;

    float r2 = r * r;
    float r25 = r2 * std::sqrt(r);
    float r3 = r * r2;
    float r4 = r2 * r2;
    float r45 = r4 * std::sqrt(r);
    float r5 = r2 * r3;

    float ui2 = ui * ui;
    float uo2 = uo * uo;
    float sqrtuiuo = std::sqrt((1.0 - ui2) * (1.0 - ui2));

    float C100 = 1.0 + (-0.1 * r + 0.84 * r4) / (1.0 + 9.0 * r3);
    float C101 = (0.0173 * r + 20.4 * r2 - 9.47 * r3) / (1.0 + 7.46 * r);
    float C102 = (-0.927 * r + 2.37 * r2) / (1.24 + r2);
    float C103 = (-0.11 * r - 1.54 * r2) / (1.0 - 1.05 * r + 7.1 * r2);
    float f10 = ((C100 + C101 * ui * uo + C102 * ui2 * uo2 + C103 * (ui2 + uo2)) * sigio) / sigdenom;

    float C110 = (0.54 * r - 0.182 * r3) / (1. + 1.32 * r2);
    float C111 = (-0.097 * r + 0.62 * r2 - 0.375 * r3) / (1. + 0.4 * r3);
    float C112 = 0.283 + 0.862 * r - 0.681 * r2;
    float f11 = (sqrtuiuo * (C110 + C111 * ui * uo)) * std::pow(sigio, C112) / sigdenom;

    float C200 = (0.00056 * r + 0.226 * r2) / (1. + 7.07 * r2);
    float C201 = (-0.268 * r + 4.57 * r2 - 12.04 * r3) / (1. + 36.7 * r3);
    float C202 = (0.418 * r + 2.52 * r2 - 0.97 * r3) / (1. + 10. * r2);
    float C203 = (0.068 * r - 2.25 * r2 + 2.65 * r3) / (1. + 21.4 * r3);
    float f20 = (C200 + C201 * ui * uo + C203 * ui2 * uo2 + C202 * (ui + uo) + C204 * (ui2 + uo2)) / (ui +uo);

    float C210 = (-0.049 * r - 0.027 * r3) / (1. + 3.36 * r2);
    float C211 = (2.77 * r2 - 8.332 * r25 + 6.073 * r3) / (1. + 50. * r4);
    float C212 = (-0.431 * r2 - 0.295 * r3) / (1. + 23.9 * r3);
    float f21 = (sqrtuiuo * (C210 + C211 * ui * uo + C212 * (ui + uo))) / (ui + uo);

    float C300 = (-0.083 * r3 + 0.262 * r4) / (1. - 1.9 * r2 + 38.6 * r4);
    float C301 = (-0.627 * r2 + 4.95 * r25 - 2.44 * r3) / (1. + 31.5 * r4);
    float C302 = (0.33 * r2 + 0.31 * r25 + 1.4 * r3) / (1. + 20. * r3);
    float C303 = (-0.74 * r2 + 1.77 * r25 - 4.06 * r3) / (1. + 215 * r5);
    float f30 = (C300 + C301 * ui * uo + C303 * ui2 * uo2 + C302 * (ui + uo) + C304 * (ui2 + uo2)) / (ui + uo);

    float C310 = (0.028 * r2 - 0.0132 * r3) / (1. + 7.46 * r2 - 3.315 * r4);
    float C311 = (-0.134 * r2 + 0.162 * r25 + 0.302 * r3) / (1. + 57.5 * r45);
    float C312 = (-0.119 * r2 + 0.5 * r25 - 0.207 * r3) / (1. + 18.7 * r3);
    float f31 = (sqrtuiuo * (C310 + C311 * ui * uo + C312 * (ui + uo))) / (ui + uo);

    return std::numbers::inv_pi_v<float> * (c * std::max(0.f, f10 + f11 * std::cos(phi) * 2. + f12 * std::cos(2.0 * phi) * 2.) +
        c * c * std::max(0.f, f20 + f21 * std::cos(phi) * 2. +
        c * c * c * std::max(0.f, f30 + f31 * std::cos(phi) * 2.)) +
        fm(ui, uo, r, c);
}

vec3f vmf_diffuse_albedo_mapping(vec3f kd, float r) {
    float r2 = r * r;
    float s = 0.64985f + 0.631112f * r + 1.38922f * r2;
    float sqrt_r = std::sqrt(r);
    return (-1.f + kd + std::sqrt(1.f - 2. * kd + kd * kd + 4.f * s * s * kd * kd)) / (2.f * s * kd) * sqrt_r + (1.f - sqrt_r) * kd;
}

// BSDF part
struct vMFDiffuseBSDF {
    AtVector N;
    AtVector Ng, Ns;
    float roughness;
};

AI_BSDF_EXPORT_METHODS(vMFDiffuseBSDFMtd);

bsdf_init {
    vMFDiffuseBSDF *data = (vMFDiffuseBSDF*) AiBSDFGetData(bsdf);
    data->Ns = (sg->Ngf == sg->Ng) ? sg->Ns : -sg->Ns;
    data->Ng = sg->Ngf;
    static const AtBSDFLobeInfo lobe_info[1] = {
        {AI_RAY_DIFFUSE_REFLECT, 0, AtString()}
    };
    AiBSDFInitLobes(bsdf, lobe_info, 1);
    AiBSDFInitNormal(bsdf, data->N, true);
}

bsdf_eval {
    auto *data = (vMFDiffuseBSDF*)AiBSDFGetData(bsdf);

    // microflake model should not do this.
    // but since vMF diffuse now uses lambertian sampling,
    // we suppose no ray transmit through the surface
    // not sure if my understanding is correct
    const float ui = AiV3Dot(data->N, wi);
    if (ui <= 0.f)
        return AI_BSDF_LOBE_MASK_NONE;

    // prepare need values
    AtShaderGlobals* sg = AiShaderGlobals();
    const auto wo = -sg->Rd;
    const float uo = AiV3Dot(data->N, wo);
    AtVector tagent, bitangent;
    AiV3BuildLocalFrame(tagent, bitangent, data->N);
    // need some extra tool for vector handling, wether in AtVector?

    const float weight = AiBSDFBumpShadow(data->Ns, data->N, wi) * 
}

bsdf_sample {

}

AtBSDF* vMFDiffuseBSDFCreate(const AtShaderGlobals* sg, const AtRGB& weight, const AtVector& N, float roughness) {
    AtBSDF* bsdf = AiBSDF(sg, weight, vMFDiffuseBSDFMtd, sizeof(vMFDiffuseBSDF));
    vMFDiffuseBSDF* data = (vMFDiffuseBSDF*) AiMalloc(sizeof(vMFDiffuseBSDF));
    data->N = N;
    data->roughness = roughness;
    return bsdf;
}

// shader part
AI_SHADER_NODE_EXPORT_METHODS(vMFDiffuseMethods)

enum {
    p_roughness
};

struct vMFDiffuseData {
    AtParamValue roughness;
};

node_parameters {
    AiParameterFlt("roughness", 0.5f);
}

node_initialize {
    vMFDiffuseData* data = (vMFDiffuseData*) AiMalloc(sizeof(vMFDiffuseData));
    AiNodeSetLocalData(node, data);
}

node_update {
    vMFDiffuseData* data = (vMFDiffuseData*) AiNodeGetLocalData(node);
    data->roughness = AiNodeGetParamFlt(node, "roughness");
}

node_finish {
    AiFree(AiNodeGetLocalData(node));
}

shader_evaluate {

}

node_loader {
    if (i > 0) return false;

    node->methods = vMFDiffuseMethods;
    node->output_type = AI_TYPE_RGB;
    node->name = "vMF_diffuse";
    node->node_type = AI_NODE_SHADER;
    strcpy(node->version, AI_VERSION);
    return true;
}