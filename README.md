
# lajolla
Personal modified version based on public Lajolla renderer

# Build
All the dependencies are included. Use CMake to build.
If you are on Unix systems, try
```
mkdir build
cd build
cmake ..
cmake --build .
```
It requires compilers that support C++17 (gcc version >= 8, clang version >= 7, Apple Clang version >= 11.0, MSVC version >= 19.14).

Apple M1 users: you might need to build Embree from scratch since the prebuilt MacOS binary provided is built for x86 machines. (But try build command above first.)

# Run
Try 
```
cd build
./lajolla ../scenes/cbox/cbox.xml
```
This will generate an image "image.exr".

To view the image, use [hdrview](https://github.com/wkjarosz/hdrview), or [tev](https://github.com/Tom94/tev).

# Acknowledgement
The renderer is heavily inspired by [pbrt](https://pbr-book.org/), [mitsuba](http://www.mitsuba-renderer.org/index_old.html), and [SmallVCM](http://www.smallvcm.com/).

We use [Embree](https://www.embree.org/) for ray casting.

We use [pugixml](https://pugixml.org/) to parse XML files.

We use [pcg](https://www.pcg-random.org/) for random number generation.

We use [stb_image](https://github.com/nothings/stb) and [tinyexr](https://github.com/syoyo/tinyexr) for reading & writing images.

We use [miniz](https://github.com/richgel999/miniz) for compression & decompression.

We use [tinyply](https://github.com/ddiakopoulos/tinyply) for parsing PLY files.

Many scenes in the scenes folder are directly downloaded from [http://www.mitsuba-renderer.org/download.html](http://www.mitsuba-renderer.org/download.html). Scenes courtesy of Wenzel Jakob, Cornell Program of Computer Graphics, Marko Dabrovic, Eric Veach, Jonas Pilo, and Bernhard Vogl.


## New Feature: Volume Path Tracing

## New Feature: Soft Shadow terminator (for low-poly models)

## New Feature: Character Rigging and SkVinning

Currently the project adopts LBS for character pose setup, from bone construction to rigging to the skinned mesh:

<table align="center">
<tr><th align="center">Bone Setup</th><th align="center">Rigging</th><th align="center">Skinning Result</th></tr>
<tr>
<td align="center"><img src="project/images/testbone.jpg" height="250"></td>
<td align="center"><img src="project/images/testrig.jpg" height="250"></td>
<td align="center"><img src="project/images/testskin.jpg" height="250"></td>
</tr>
</table>

To render stylized characters in the path tracer, the geometry must first be posed from a rest pose into an arbitrary animation frame. This is the job of **skeletal skinning**: each mesh vertex is bound to one or more bones, and a skinning algorithm decides how those bone transformations combine to produce the final vertex position. The choice of skinning method has a direct impact on visual quality around joints — artifacts such as volume collapse ("candy-wrapper"), unnatural bulging, or rigid-looking bends are all consequences of how the underlying math blends transformations. Below we survey and compare four widely-used skinning strategies in increasing order of sophistication.

<p align="center">
<img src="project/images/diagramSkinning.png" width="574">
</p>

### Linear Blend Skinning (LBS)

Linear Blend Skinning is the industry baseline due to its simplicity and speed. Each vertex $\mathbf{v}$ in the rest pose is transformed by a weighted average of bone transformation matrices:

$$\mathbf{v}' = \sum_{i=1}^{n} w_i \, M_i \, \mathbf{v}$$

where $M_i$ is the world-space transformation matrix of bone $i$ and $w_i$ is the corresponding skinning weight with $\sum w_i = 1$. Because the blending operates directly on matrix entries (i.e. on Cartesian coordinates), the interpolated path between two bone poses is a straight line through space. This is fast but geometrically problematic: when a joint bends significantly, the straight-line interpolation causes vertices near the joint to collapse inward, producing the well-known **candy-wrapper artifact**. The volume around the joint shrinks visibly, especially on forearms under twist and tightly bent knees. Despite this, LBS remains the default in most real-time engines because of its trivial GPU implementation.

### Dual Quaternion Skinning (DQS)

Dual Quaternion Skinning addresses the volume-collapse problem of LBS by representing each bone transformation as a **dual quaternion** $\hat{q} = q_0 + \varepsilon \, q_\varepsilon$, encoding both rotation ($q_0$) and translation ($q_\varepsilon$) in a unified 8-component form. The blended transformation is computed by averaging and normalizing in dual quaternion space:

$$\hat{q}_{\text{blend}} = \frac{\sum_{i=1}^{n} w_i \, \hat{q}_i}{\left\| \sum_{i=1}^{n} w_i \, \hat{q}_i \right\|}$$

The vertex is then transformed by converting $\hat{q}_{\text{blend}}$ back to a rigid-body transformation. Because quaternion interpolation traces a **spherical arc** (on the 4D unit sphere) rather than a straight line, the blended transformation remains a proper rigid motion and **preserves volume** around bent joints. However, this rigidity comes with a trade-off: at sharp bend angles, the spherical interpolation can overshoot, causing vertices on the inside of the bend to **bulge outward** unnaturally. DQS is slightly more expensive than LBS but the cost is modest and well within real-time budgets.

### Bulge-Free Dual Quaternion Skinning (Bulge-Free DQS)

Bulge-Free DQS refines standard DQS by applying a **scaling correction** on top of the rigid blend. After computing $\hat{q}_{\text{blend}}$ as above, the method constructs a corrective scale/shear component $S_{\text{corr}}$ that gently compresses the geometry in the direction of the bulge:

$$\mathbf{v}' = T(\hat{q}_{\text{blend}}) \cdot S_{\text{corr}} \cdot \mathbf{v}$$

where $T(\hat{q}_{\text{blend}})$ is the rigid transformation from the normalized dual quaternion, and $S_{\text{corr}}$ is derived from the difference between the DQS result and a reference pose. In effect, this produces **damped DQS**: it intentionally allows a small, controlled amount of LBS-style "collapse" to counteract the DQS bulge, finding a middle ground between the two extremes. The result looks natural at moderate bend angles and avoids the most objectionable artifacts of both pure LBS and pure DQS.

### Optimized Center of Rotation Skinning (Disney OCoR)

Optimized Center of Rotation (OCoR) skinning, introduced by researchers at Disney, takes a fundamentally different geometric approach. Instead of blending transformations, OCoR computes a **unique pivot point** $\mathbf{c}_j$ for each vertex $j$ based on the skinning weights and bone rest positions:

$$\mathbf{c}_j = \frac{\sum_{i=1}^{n} \sum_{k=1}^{n} w_{j,i} \, w_{j,k} \left( \frac{\mathbf{p}_i + \mathbf{p}_k}{2} \right)}{\sum_{i=1}^{n} \sum_{k=1}^{n} w_{j,i} \, w_{j,k}}$$

where $\mathbf{p}_i$ is the rest-pose position of bone $i$. This pivot is the weighted center of all pairwise midpoints of the influencing bones. The vertex is then rotated around this per-vertex pivot using a linear blend of rotational components:

$$\mathbf{v}_j' = \left( \sum_{i=1}^{n} w_{j,i} \, R_i \right) (\mathbf{v}_j - \mathbf{c}_j) + \sum_{i=1}^{n} w_{j,i} \, (R_i \, \mathbf{c}_j + \mathbf{t}_i)$$

The result is **anatomically plausible** deformation: because each vertex rotates around its own optimized center rather than a shared joint, the straight-line interpolation of LBS is redirected to follow a curved path approximating the true arc of rotation. OCoR achieves this at essentially the same runtime cost as LBS (the per-vertex pivots are precomputed once during rigging), naturally avoiding both the candy-wrapper collapse and the DQS bulge.

### Comparison

To summarize, **LBS** averages coordinates linearly and is the fastest, but its straight-line interpolation path causes the candy-wrapper collapse at bent joints. **DQS** averages transformations spherically, preserving volume via a proper arc on the quaternion sphere, yet this rigidity introduces bulging at sharp bends. **Bulge-Free DQS** augments DQS with a corrective scale factor, intentionally allowing a small amount of collapse to suppress the bulge — a damped middle ground. **Disney OCoR** sidesteps the blending problem entirely by computing a unique rotation pivot per vertex, so even though it uses LBS-speed linear math, the displaced center of rotation makes the straight-line path approximate a curve, yielding anatomically plausible deformation without either collapse or bulge. In this project, the skinning step is performed as a pre-process before feeding the posed mesh to the path tracer, so the choice of algorithm directly determines the quality of the input geometry that Lajolla ultimately renders.

## New Feature: Fuse NPR in BPR path tracing pipeline
```
# Render the room stage
lajolla.exe ../../scenes/NPBPR/simpleShufang_conv.xml -o Shufangimage.exr
# Render the NPR character
lajolla.exe ../../scenes/NPR/nprxmltest.xml -o toon_image.exr
```

## Abstract

This project integrates non-photorealistic rendering techniques, including cel shading via toon/sphere matcaps, quantized cosine shading, and Fresnel-term hacking on highlight and rim, into lajolla renderer. And then demonstrate how these modifications can coexist with PBR materials to produce stylized yet physically grounded renderings.

---

## 1. Introduction

NPR like cel shading are widely used in anime and game production but are traditionally implemented in rasterization pipelines, lacking global illumination effects such as soft shadows and indirect lighting which would be interesting sometimes. This project explores integrating NPR into a Monte Carlo path tracer, enabling stylized shading that coexists with physically-based materials and lighting in a unified rendering framework.


## 2. Background

### 2.1 Physics-Based Rendering (PBR)

The project is currently built on Lajolla path tracer (no volume so far), and extend disney principled BSDF to support NPR materials. Theoretically, we can assigne all objects in the scene with one material, if ignoring rendering time and some effort of formatting convertion. Before that, I add some numerical stability tricks to the renderer, handling zero-initialization, wild-pointer and division-by-zero, etc. This is to enusure the code doesn't crash when runnign on Windows Release optimization mode. Meanwhile, I add some light types such as ambient, emission shader, handling shadow terminator to make sure low-poly models can be rendered properly. Those can be found partially in checkpoint report. 


### 2.2 Non-Photorealistic Rendering (NPR)

In order to implement NPR, the main two lobes needing modification are disneydiffuse and disneytintmetal (hw.2 version + speculat tint). Also disneyglass blends with other lobes in a specific way, so I also need to add the final transparent bsdf for some NPR-sttle control. The project so far has two main NPR styles: quantized cosine shading (hard-edge cel shading, specualr higlight, rim lighting) and toon/sphere matcap shading (often used in 3D-to-2D rendering and mobile games to achive some view-dependent stylized shading with cheaper computation cost). Those will be introduced later.



### 2.3 Matcaps (Toon & Sphere Maps)
Toon and sphere matcaps (material captures) are pre-painted textures that encode lighting response as a function of the surface normal in view space. Toon matcaps store discrete color bands for cel shading; sphere matcaps store smooth highlight gradients. Both are view-dependent and light-independent.
Detailed equation is shown below.

## 3. Method

### 3.1 Toon and Sphere Matcap Integration

Given a surface with shading normal **n** in world space, we transform it to view space using the camera's world-to-camera matrix:

$$\mathbf{n}_v = \text{normalize}(M_{\text{view}} \cdot \mathbf{n})$$

The matcap UV is then:

$$u = n_{v,x} \cdot 0.5 + 0.5, \quad v = n_{v,y} \cdot 0.5 + 0.5$$

The toon matcap $T(u,v)$ and sphere matcap $S(u,v)$ are sampled at these coordinates. The final NprBSDF combines them with the base diffuse and glossy (tint-metal) lobes:

$$f_{\text{npr}} = \Big[(1 - F_1) \cdot (f_{\text{diffuse}} \cdot T + S) + F_1 \cdot f_{\text{glossy}}\Big] \cdot F_2 + (1 - F_2) \cdot f_{\text{transparent}}$$

where $F_1$ (`Fac`) blends between diffuse+matcap and glossy metallic, and $F_2$ (`Fac2`) controls opacity — blending between the opaque shading and a straight-through transparent BSDF. The toon matcap multiplies the diffuse lobe to create extra layer of apperance variance, while the sphere matcap adds view-dependent highlights additively. Detailedly, I create a new NPR_diffuse BSDF in the code to take those toon and sphere texture lookups as input, without the need of passing the tuxreture map to save comutation. Note because the shpere part is addition, this breaks the linearilty of Diffuse lobe:
$Diffuse(toon*basecolor)=toon*Diffuse(basecolor)$ but $Diffuse(toon*basecolor+sphere)=toon*Diffuse(basecolor)+sphere$ is not true, the shader needs texture as basecolor, but toon and shpere is view-dependent, the calucaltion is dynamic if doing so and becoems slow.

To demonstrate the effect of toon and sphere matcaps, below are typical scnenes in cloths. Those parts effectively add some view-varying layer of reflection on top of the base color.

<table align="center">
<tr><th align="center">diffuse+metal</th><th align="center">add Toon matcap</th><th align="center">and add Sphere matcap</th></tr>
<tr>
<td align="center"><img src="project/images/stoc1.png" height="250"></td>
<td align="center"><img src="project/images/stoc1toon.png" height="250"></td>
<td align="center"><img src="project/images/stoc1sphere.png" height="250"></td>
</tr>
</table>

<table align="center">
<tr><th align="center">diffuse+metal</th><th align="center">add Toon matcap</th><th align="center">and add Sphere matcap</th></tr>
<tr>
<td align="center"><img src="project/images/stoc2.png" height="250"></td>
<td align="center"><img src="project/images/stoc2toon.png" height="250"></td>
<td align="center"><img src="project/images/stoc2sphere.png" height="250"></td>
</tr>
</table>


Similarly, the shere matcap can mimic a cel-shading effect, if you apply a sharp edge sphere matcap. So this is often a cheap budget to mimic and reproduce those complex ray interactions.

<table align="center">
<tr><th align="center">no matcap</th><th align="center">sphere matcap</th></tr>
<tr>
<td align="center"><img src="project/images/shoo.png" height="250"></td>
<td align="center"><img src="project/images/shoosphere.png" height="250"></td>
</tr>
</table>

### 3.2 NPR Shading: Quantized Cosine to modify BSDFs

The classic cel-shading approach quantizes the diffuse cosine term into discrete bands. Instead of smooth Lambertian shading $\mathbf{N} \cdot \mathbf{L}$, it applies a step function:

$$f_{\text{cel}} = K_a + K_d \cdot \text{step}(\mathbf{N} \cdot \mathbf{L})$$

where $\text{step}(x) = \begin{cases} 1 & x > x_c \\ 0 & \text{otherwise} \end{cases}$, producing a hard two-tone shading. A specular highlight is added using the half-vector $\mathbf{H} = \text{normalize}(\mathbf{V} + \mathbf{L})$:

$$f_{\text{spec}} = \text{step}\big((\mathbf{N} \cdot \mathbf{H})^s - t_{\text{spec}}\big)$$

where $s$ is the shininess exponent and $t_{\text{spec}}$ is a threshold controlling highlight size. Rim lighting simulates edge glow using the view-normal falloff:

$$f_{\text{rim}} = (1 - \mathbf{V} \cdot \mathbf{N}) \cdot (\mathbf{N} \cdot \mathbf{L})^{0.1}$$

The full shading is then:

$$L_o = f_{\text{cel}} \cdot C_{\text{base}} + f_{\text{spec}} \cdot C_{\text{spec}} + f_{\text{rim}} \cdot C_{\text{rim}}$$

All these are just input to the diffuse base_color, as stated in the last section. In standard rasterization, these are evaluated per-light in a forward pass. In Lajolla path tracer, I instead hack the Fresnel/BSDF evaluation to produce the same quantized response while still benefiting from ray-traced shadows and indirect lighting. 

Below are comparisons of cel shdaing in rasterization (true NPR), PBR with physical BSDF, and their fusion that may tell the interest of combing  NPR and PBR. Noticeably, the fused version can show the soft shadows and indirect lighting from the environment, which is not possible in pure rasterization NPR.


<table align="center">
<tr><th align="center">NPR Style 1</th><th align="center">NPR Style 2</th></tr>
<tr>
<td align="center"><img src="project/images/npr.png" height="250"></td>
<td align="center"><img src="project/images/npr_.png" height="250"></td>
</tr>
</table>

<table align="center">
<tr><th align="center">Fuse Style 1</th><th align="center">Fuse Style 2</th></tr>
<tr>
<td align="center"><img src="project/images/fuse.png" height="250"></td>
<td align="center"><img src="project/images/fuse_.png" height="250"></td>
</tr>
</table>


<table align="center">
<tr><th align="center">PBR (Ray Tracing)</th><th align="center">NPR (Quantized Cosine)</th><th align="center">Fuse (Hacked Fresnel)</th></tr>
<tr>
<td align="center"><img src="project/images/pbr.png" height="250"></td>
<td align="center"><img src="project/images/npr.png" height="250"></td>
<td align="center"><img src="project/images/fuse.png" height="250"></td>
</tr>
</table>

### 3.3 Specular Highlight, Rim Lighting and Outline

Below are some examples of specular highlight, rim lighting and outline. Based on the quation, each part can take different colors of virtual lighting. So I make speucalr in sunset color and rim in more bright red. Interstingly, I initially mis flip the gradient of normal-view cosine field, the specular comes to appear in shadow, but looks good.
For the outline, I use a very simple trick, dilating the object and useing backface culling. This makes redenring a bit messy than other post process method, so this canbe further improved.

<table align="center">
<tr><th align="center">Highlight Style 1</th><th align="center">Highlight Style 2</th></tr>
<tr>
<td align="center"><img src="project/images/highlight.png" height="250"></td>
<td align="center"><img src="project/images/highlight_.png" height="250"></td>
</tr>
</table>

<table align="center">
<tr><th align="center">Highlight Inv 1</th><th align="center">Highlight Inv 2</th><th align="center">Highlight Inv 3</th><th align="center">Highlight Inv 4</th></tr>
<tr>
<td align="center"><img src="project/images/highlight_inv.png" height="250"></td>
<td align="center"><img src="project/images/highlight_inv_.png" height="250"></td>
<td align="center"><img src="project/images/highlight_inv__.png" height="250"></td>
<td align="center"><img src="project/images/highlight_inv___.png" height="250"></td>
</tr>
</table>

<table align="center">
<tr><th align="center">Rim Style 1</th><th align="center">Rim Style 2</th></tr>
<tr>
<td align="center"><img src="project/images/rim.png" height="250"></td>
<td align="center"><img src="project/images/rim_.png" height="250"></td>
</tr>
</table>

<table align="center">
<tr>
<th align="center">Outline</th>
<td rowspan="2" align="center"><img src="project/images/NPR2.jpg" height="300"></td>
</tr>
<tr>
<td align="center"><img src="project/images/outline.png" height="250"></td>
</tr>
</table>

---

## 4. Scene Integration: Unifying PBR and NPR

### 4.1 PBR Scene Without Character

First built a PBR stage in Lajolla (this takes so many time actually :)), next time might come with a stronger script for format conversion but the good thing now is it is all using disney principled BSDF. Then I tried to put the character in the scene, but it turns out the lighting is not the same for NPR BSDF, and current state only includes the toon and sphere matcap, a combined test of cosine quantization can look dim right now. 

**All scenes rendered just by Lajolla renderer!**
<table align="center">
<tr><th align="center">PBR Scene (no character)</th><th align="center">Unified PBR + toon.sphere matcap</th></tr>
<tr>
<td align="center"><img src="project/images/noCharac.png" height="250"></td>
<td align="center"><img src="project/images/unify.png" height="250"></td>
</tr>
</table>

Because the shader is now not disney principled BSDF, the renderign style looks some different. Continue work on this will include above test scene effect in hilight and rim lighting. 
<!--A code linke keep tracks of my update: https://github.com/Furkath/Lajolla_mod.-->
PS: processing the scene and rigging the character takes so much time, I will try to make a script to automate this process later.

---

## 5. Implementation Details

### Below are some code snippets of my implementation. I would update the code repository with more details later, which just compile and run should reproduce the image of this report.

#### Project Structure

The scene assets are organized under `scenes/NPBPR/`, containing mesh files (`.ply`), texture images, and XML scene descriptions. The source code extends the `src/materials/` directory with new files: `npr_bsdf.inl` for the main NPR BSDF logic, and `npr_diffuse.inl` for the modified diffuse lobe. Existing Disney BSDF files are preserved alongside backup and annotated (`note_`) versions for reference.

<table align="center">
<tr><th align="center">Scene Assets</th><th align="center">Source Structure</th></tr>
<tr>
<td align="center"><img src="project/images/屏幕截图 2026-03-20 000300.png" height="350"></td>
<td align="center"><img src="project/images/屏幕截图 2026-03-20 000327.png" height="350"></td>
</tr>
</table>

#### XML Scene Description (NPR Material)

The NPR BSDF is defined in XML with bitmap textures for `diffuse_color`, `toon_color`, and `sphere_color`, along with float parameters controlling metallic blending (`Fac`), opacity (`Fac2`), roughness, and a `color_scale` multiplier. PBR materials (Disney diffuse/sheen) coexist in the same scene file. Emitters are defined as point lights and constant-radiance sources.

```xml
<!-- NPR Material -->
<bsdf type="npr" id="mat-face" name="mat-face">
    <texture type="bitmap" name="diffuse_color">
        <string name="filename" value="textures/face.png"/>
    </texture>
    <texture type="bitmap" name="toon_color">
        <string name="filename" value="textures/skin.bmp"/>
    </texture>
    <texture type="bitmap" name="sphere_color">
        <string name="filename" value="textures/sphere.bmp"/>
    </texture>
    <float name="color_scale" value="2"/>
    <rgb name="metallic_color" value="0.00, 0.00, 0.00"/>
    <float name="Fac" value="0.02"/>
    <float name="diffuse_roughness" value="0.0"/>
    <float name="metallic_roughness" value="1.0"/>
</bsdf>

<!-- PBR Material in same scene -->
<bsdf type="npr" id="mat-caishi" name="mat-caishi">
    <texture type="bitmap" name="diffuse_color">
        <string name="filename" value="textures/ImageBIND.006.png"/>
    </texture>
    <float name="color_scale" value="1.5"/>
    <rgb name="metallic_color" value="0.58, 0.48, 0.56"/>
    <float name="Fac" value="0.0897"/>
    <float name="diffuse_roughness" value="0.89067160"/>
    <float name="metallic_roughness" value="0.08"/>
    <float name="fac2" value="0.530013"/>
</bsdf>
```

#### Scene Parser: NPR BSDF Initialization (`parse_scene.cpp`)

When the parser encounters `type == "nprbsdf" || type == "npr"`, it initializes all NPR-specific textures and parameters with sensible defaults. Bitmap info structs track `diffuse_bmp`, `toon_bmp`, and `sphere_bmp` for matcap lookup. The `fac2` parameter defaults to 1.0 (fully opaque) unless explicitly set.

```cpp
} else if (type == "nprbsdf" || type == "npr") {
    Texture<Spectrum> diffuse_color = make_constant_spectrum_texture(fromRGB(Vector3(0.5, 0.5, 0.5)));
    Texture<Spectrum> metallic_color = make_constant_spectrum_texture(fromRGB(Vector3(0.5, 0.5, 0.5)));
    Texture<Spectrum> toon_color = make_constant_spectrum_texture(fromRGB(Vector3(1, 1, 1)));
    Texture<Spectrum> sphere_color = make_constant_spectrum_texture(fromRGB(Vector3(0, 0, 0)));
    Texture<Real> fac = make_constant_float_texture(Real(0));
    Texture<Real> fac2 = make_constant_float_texture(Real(1));    // opacity: 1 = opaque
    Texture<Real> metallic = make_constant_float_texture(Real(1.0));
    Texture<Real> diffuse_roughness = make_constant_float_texture(Real(0.5));
    Texture<Real> metallic_roughness = make_constant_float_texture(Real(0.5));
    Texture<Real> subsurface = make_constant_float_texture(Real(0));
    Texture<Real> specular = make_constant_float_texture(Real(0.5));
    Texture<Real> specular_tint = make_constant_float_texture(Real(0));
    Texture<Real> anisotropic = make_constant_float_texture(Real(0));
    Real color_scale = Real(1);
    Real eta = Real(1.5);
    bool fac2_explicitly_set = false;
    // Track bitmap filenames and UV params for automatic alpha extraction
    struct BitmapInfo { fs::path filename; Real us = 1, vs = 1, uo = 0, vo = 0; };
    BitmapInfo diffuse_bmp, toon_bmp, sphere_bmp;
    // Helper: extract bitmap info from a texture node
    auto extract_bitmap_info = [&](pugi::xml_node child, BitmapInfo &info) { ... };
    ...
}
```

#### NPR BSDF Core: Matcap UV & Evaluation (`npr_bsdf.inl`)

The `compute_toon_uv` function transforms the shading normal into view space to produce matcap UVs. The `eval_op::operator()` evaluates the full NPR shading model: it samples toon and sphere matcaps, computes the NPR diffuse and Disney tint-metal lobes, and blends them according to the `Fac`/`Fac2` formula.

```cpp
// Compute matcap UV from shading normal and incoming ray direction
// Projects shading normal to camera-aligned frame, maps x,y from [-1,1] to [0,1]
inline Vector2 compute_toon_uv(const Vector3 &shading_normal, const Vector3 &dir_in) {
    Real len_sq = dot(dir_in, dir_in);
    if (len_sq < Real(1e-10)) {
        return Vector2(Real(0.5), Real(0.5));
    }
    Vector3 view_z = dir_in / sqrt(len_sq);
    Vector3 world_up(0, 1, 0);
    if (fabs(dot(view_z, world_up)) > Real(0.999)) {
        world_up = Vector3(1, 0, 0);
    }
    Vector3 cross_val = cross(world_up, view_z);
    Real cross_len_sq = dot(cross_val, cross_val);
    if (cross_len_sq < Real(1e-10)) {
        return Vector2(Real(0.5), Real(0.5));
    }
    Vector3 view_x = cross_val / sqrt(cross_len_sq);
    Vector3 view_y = cross(view_z, view_x);
    Real u = std::clamp(dot(shading_normal, view_x) * Real(0.5) + Real(0.5), Real(0), Real(1));
    Real v = std::clamp(dot(shading_normal, view_y) * Real(0.5) + Real(0.5), Real(0), Real(1));
    return Vector2(u, v);
}

Spectrum eval_op::operator()(const NprBSDF &bsdf) const {
    Vector2 toon_uv = compute_toon_uv(vertex.shading_frame.n, dir_in);
    Real fac2 = eval(bsdf.Fac2, vertex.uv, vertex.uv_screen_size, texture_pool)
              * eval(bsdf.toon_alpha, toon_uv, vertex.uv_screen_size, texture_pool)
              * eval(bsdf.sphere_alpha, toon_uv, vertex.uv_screen_size, texture_pool);
    bool front_face = dot(vertex.geometric_normal, dir_in) > 0;
    // If backfacing, just see through
    if (!front_face) fac2 = 0;

    Spectrum total = make_zero_spectrum();

    // Opaque lobes (front face only)
    if (fac2 > 0) {
        Real fac = eval(bsdf.Fac, vertex.uv, vertex.uv_screen_size, texture_pool);
        Spectrum toon = eval(bsdf.toon_color, toon_uv, vertex.uv_screen_size, texture_pool);
        Spectrum sphere = eval(bsdf.sphere_color, toon_uv, vertex.uv_screen_size, texture_pool);

        NprDiffuse diffuse_lobe = {bsdf.diffuse_color, bsdf.diffuse_roughness, bsdf.subsurface,
                                    toon, sphere};
        DisneyTintMetal metal_lobe = {bsdf.metallic_color, bsdf.metallic_roughness, bsdf.anisotropic,
                                       bsdf.specular, bsdf.specular_tint, bsdf.metallic, bsdf.eta};
        Spectrum opaque = make_zero_spectrum();
        // opaque = ((1-Fac)*(diffuse*toon+sphere) + Fac*metal) * Fac2 + (1-Fac2) * transparent
        ...
    }
    ...
}
```

---

