struct Uniforms {
    view_projection: mat4x4<f32>,
    pivot: vec4<f32>,
    angle: vec4<f32>,
}
@group(0) @binding(0) var<uniform> uniforms: Uniforms;
struct Vertex {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec3<f32>,
    @location(3) moving: f32,
}
struct Fragment {
    @builtin(position) position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) color: vec3<f32>,
}
@vertex fn vertex(input: Vertex) -> Fragment {
    let a = uniforms.angle.x * input.moving;
    let c = cos(a);
    let s = sin(a);
    let rotation = mat3x3<f32>(vec3(c,0,-s),vec3(0,1,0),vec3(s,0,c));
    let position = rotation * (input.position - uniforms.pivot.xyz) + uniforms.pivot.xyz;
    var output: Fragment;
    output.position = uniforms.view_projection * vec4(position,1);
    output.normal = rotation * input.normal;
    output.color = input.color;
    return output;
}
@fragment fn fragment(input: Fragment) -> @location(0) vec4<f32> {
    let light = 0.35 + 0.65 * abs(dot(normalize(input.normal), normalize(vec3(0.3,0.8,0.5))));
    return vec4(input.color * light, 1);
}
