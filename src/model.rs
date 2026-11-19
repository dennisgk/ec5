use eframe::{
    egui,
    egui_wgpu::{self, wgpu},
};
use glam::{Mat4, Vec3};
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
    color: [f32; 3],
    moving: f32,
}
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    matrix: [[f32; 4]; 4],
    pivot: [f32; 4],
    angle: [f32; 4],
}

pub struct Model {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::RenderPipeline,
    vertices: wgpu::Buffer,
    count: u32,
    uniform: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    color: wgpu::TextureView,
    depth: wgpu::TextureView,
    pub texture: egui::TextureId,
    pivot: Vec3,
    center: Vec3,
    radius: f32,
}
impl Model {
    pub fn new(state: &egui_wgpu::RenderState) -> Result<Self, String> {
        let gltf = gltf::Gltf::from_slice(include_bytes!("../assets/brachiflex.glb"))
            .map_err(|e| e.to_string())?;
        let blob = gltf
            .blob
            .as_deref()
            .ok_or("Model has no embedded geometry")?;
        let mut vertices = Vec::new();
        let mut pivot = Vec3::ZERO;
        let mut low = Vec3::splat(f32::MAX);
        let mut high = Vec3::splat(f32::MIN);
        for node in gltf.nodes() {
            let transform = Mat4::from_cols_array_2d(&node.transform().matrix());
            if node.name() == Some("ElbowPivot") {
                pivot = transform.transform_point3(Vec3::ZERO);
            }
            let Some(mesh) = node.mesh() else {
                continue;
            };
            let moving = if node.name().is_some_and(|name| name.starts_with("Moving")) {
                1.0
            } else {
                0.0
            };
            for primitive in mesh.primitives() {
                if primitive.mode() != gltf::mesh::Mode::Triangles {
                    continue;
                }
                let reader = primitive.reader(|_| Some(blob));
                let positions: Vec<_> = reader
                    .read_positions()
                    .ok_or("Missing model positions")?
                    .collect();
                let normals: Vec<_> = reader
                    .read_normals()
                    .ok_or("Missing model normals")?
                    .collect();
                let color = primitive
                    .material()
                    .pbr_metallic_roughness()
                    .base_color_factor();
                let indices: Vec<u32> = match reader.read_indices() {
                    Some(i) => i.into_u32().collect(),
                    None => (0..positions.len() as u32).collect(),
                };
                for index in indices {
                    let p = transform.transform_point3(Vec3::from(positions[index as usize]));
                    let n = transform
                        .transform_vector3(Vec3::from(normals[index as usize]))
                        .normalize_or_zero();
                    low = low.min(p);
                    high = high.max(p);
                    vertices.push(Vertex {
                        position: p.to_array(),
                        normal: n.to_array(),
                        color: [color[0], color[1], color[2]],
                        moving,
                    });
                }
            }
        }
        if vertices.is_empty() || !pivot.is_finite() {
            return Err("Empty or invalid arm model".into());
        }
        let device = state.device.clone();
        let queue = state.queue.clone();
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Arm mesh"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let uniform = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Arm pose"),
            size: 96,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform.as_entire_binding(),
            }],
        });
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Arm shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("model.wgsl").into()),
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[&layout],
            push_constant_ranges: &[],
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label:Some("Arm renderer"),layout:Some(&pipeline_layout),
            vertex:wgpu::VertexState { module:&shader,entry_point:Some("vertex"),compilation_options:Default::default(),buffers:&[wgpu::VertexBufferLayout { array_stride:40,step_mode:wgpu::VertexStepMode::Vertex,attributes:&wgpu::vertex_attr_array![0=>Float32x3,1=>Float32x3,2=>Float32x3,3=>Float32] }] },
            fragment:Some(wgpu::FragmentState { module:&shader,entry_point:Some("fragment"),compilation_options:Default::default(),targets:&[Some(wgpu::ColorTargetState { format:wgpu::TextureFormat::Rgba8UnormSrgb,blend:None,write_mask:wgpu::ColorWrites::ALL })] }),
            primitive:wgpu::PrimitiveState { cull_mode:None,..Default::default() },
            depth_stencil:Some(wgpu::DepthStencilState { format:wgpu::TextureFormat::Depth32Float,depth_write_enabled:true,depth_compare:wgpu::CompareFunction::Less,stencil:Default::default(),bias:Default::default() }),
            multisample:Default::default(),multiview:None,cache:None,
        });
        let texture = |format, usage| {
            device
                .create_texture(&wgpu::TextureDescriptor {
                    label: Some("Arm viewport"),
                    size: wgpu::Extent3d {
                        width: 1024,
                        height: 1024,
                        depth_or_array_layers: 1,
                    },
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: wgpu::TextureDimension::D2,
                    format,
                    usage,
                    view_formats: &[],
                })
                .create_view(&Default::default())
        };
        let color = texture(
            wgpu::TextureFormat::Rgba8UnormSrgb,
            wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
        );
        let depth = texture(
            wgpu::TextureFormat::Depth32Float,
            wgpu::TextureUsages::RENDER_ATTACHMENT,
        );
        let id = state.renderer.write().register_native_texture(
            &device,
            &color,
            wgpu::FilterMode::Linear,
        );
        Ok(Self {
            device,
            queue,
            pipeline,
            vertices: vertex_buffer,
            count: vertices.len() as u32,
            uniform,
            bind_group,
            color,
            depth,
            texture: id,
            pivot,
            center: (low + high) * 0.5,
            radius: (high - low).length() * 0.55,
        })
    }
    fn view(&self) -> Mat4 {
        let r = self.radius;
        let eye = self.center + Vec3::new(r * 0.12, r * 3.0, r * 0.05);
        Mat4::perspective_rh(42_f32.to_radians(), 1.0, r * 0.01, r * 10.0)
            * Mat4::look_at_rh(eye, self.center, Vec3::Z)
    }
    pub fn draw(&self, degrees: f32) {
        let uniforms = Uniforms {
            matrix: self.view().to_cols_array_2d(),
            pivot: self.pivot.extend(1.0).to_array(),
            angle: [degrees.to_radians(), 0., 0., 0.],
        };
        self.queue
            .write_buffer(&self.uniform, 0, bytemuck::bytes_of(&uniforms));
        let mut encoder = self.device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Arm viewport"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &self.color,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.025,
                            g: 0.03,
                            b: 0.035,
                            a: 1.,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &self.bind_group, &[]);
            pass.set_vertex_buffer(0, self.vertices.slice(..));
            pass.draw(0..self.count, 0..1);
        }
        self.queue.submit([encoder.finish()]);
    }
    pub fn drag_angle(&self, start: egui::Pos2, now: egui::Pos2, rect: egui::Rect) -> f32 {
        let pivot = self.view().project_point3(self.pivot);
        let screen = egui::pos2(
            rect.left() + (pivot.x + 1.) * 0.5 * rect.width(),
            rect.top() + (1. - pivot.y) * 0.5 * rect.height(),
        );
        let a = start - screen;
        let b = now - screen;
        let delta = b.y.atan2(b.x) - a.y.atan2(a.x);
        delta.sin().atan2(delta.cos()).to_degrees()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    #[ignore = "Explicit offscreen graphics check; never accesses BLE"]
    fn render_arm_without_ble() {
        pollster::block_on(async {
            let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
                #[cfg(target_os = "windows")]
                backends: wgpu::Backends::DX12,
                #[cfg(any(target_os = "ios", target_os = "macos"))]
                backends: wgpu::Backends::METAL,
                ..Default::default()
            });
            let adapter = instance.request_adapter(&Default::default()).await.unwrap();
            let (device, queue) = adapter.request_device(&Default::default()).await.unwrap();
            let renderer = egui_wgpu::Renderer::new(
                &device,
                wgpu::TextureFormat::Rgba8UnormSrgb,
                Default::default(),
            );
            let state = egui_wgpu::RenderState {
                adapter,
                available_adapters: Vec::new(),
                device,
                queue,
                target_format: wgpu::TextureFormat::Rgba8UnormSrgb,
                renderer: std::sync::Arc::new(egui::epaint::mutex::RwLock::new(renderer)),
            };
            let model = Model::new(&state).unwrap();
            let mut pictures = Vec::new();
            for angle in [0., 60.] {
                model.draw(angle);
                let buffer = model.device.create_buffer(&wgpu::BufferDescriptor {
                    label: None,
                    size: 1024 * 1024 * 4,
                    usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                    mapped_at_creation: false,
                });
                let mut encoder = model.device.create_command_encoder(&Default::default());
                encoder.copy_texture_to_buffer(
                    model.color.texture().as_image_copy(),
                    wgpu::TexelCopyBufferInfo {
                        buffer: &buffer,
                        layout: wgpu::TexelCopyBufferLayout {
                            offset: 0,
                            bytes_per_row: Some(4096),
                            rows_per_image: Some(1024),
                        },
                    },
                    wgpu::Extent3d {
                        width: 1024,
                        height: 1024,
                        depth_or_array_layers: 1,
                    },
                );
                model.queue.submit([encoder.finish()]);
                let (tx, rx) = std::sync::mpsc::channel();
                buffer.slice(..).map_async(wgpu::MapMode::Read, move |r| {
                    let _ = tx.send(r);
                });
                model
                    .device
                    .poll(wgpu::PollType::Wait {
                        submission_index: None,
                        timeout: Some(std::time::Duration::from_secs(10)),
                    })
                    .unwrap();
                rx.recv_timeout(std::time::Duration::from_secs(10))
                    .unwrap()
                    .unwrap();
                let bytes = buffer.slice(..).get_mapped_range().to_vec();
                image::RgbaImage::from_raw(1024, 1024, bytes.clone())
                    .unwrap()
                    .save(format!("target/arm-{angle:.0}.png"))
                    .unwrap();
                pictures.push(bytes);
            }
            let differences = pictures[0]
                .chunks_exact(4)
                .zip(pictures[1].chunks_exact(4))
                .filter(|(a, b)| a != b)
                .count();
            assert!(
                differences > 500,
                "Forearm rotation must change the rendered image"
            );
        });
    }
    #[test]
    fn model_has_fixed_moving_and_elbow_nodes_without_straps() {
        let model = gltf::Gltf::from_slice(include_bytes!("../assets/brachiflex.glb")).unwrap();
        let names: Vec<_> = model.nodes().filter_map(|n| n.name()).collect();
        assert!(names.contains(&"ElbowPivot"));
        assert!(names.contains(&"MovingForearm"));
        assert!(names.contains(&"FixedAssembly"));
        assert!(
            names
                .iter()
                .all(|name| !name.to_lowercase().contains("strap"))
        );
        let blob = model.blob.as_deref().unwrap();
        let count: usize = model
            .meshes()
            .flat_map(|m| m.primitives())
            .map(|p| p.reader(|_| Some(blob)).read_positions().unwrap().count())
            .sum();
        assert!(count > 100 && count < 1_000_000);
    }
}
