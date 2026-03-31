#import bevy_pbr::forward_io::VertexOutput

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    // 100 m squares based on world XZ position.
    let scale = 0.01; // 1 / 100 m
    let gx = floor(in.world_position.x * scale);
    let gz = floor(in.world_position.z * scale);
    let checker = (i32(gx) + i32(gz)) & 1;

    let color_a = vec3<f32>(0.22, 0.48, 0.16); // lighter green
    let color_b = vec3<f32>(0.16, 0.36, 0.11); // darker green
    let color = mix(color_a, color_b, f32(checker));

    // Simple diffuse shading using the vertex normal.
    let sun_dir = normalize(vec3<f32>(0.5, 1.0, 0.5));
    let ndotl = max(dot(in.world_normal, sun_dir), 0.0);
    let ambient = 0.35;
    let lit = color * (ambient + (1.0 - ambient) * ndotl);

    return vec4<f32>(lit, 1.0);
}
