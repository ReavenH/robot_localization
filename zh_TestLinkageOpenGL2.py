import open3d as o3d

# 加载模型
mesh = o3d.io.read_triangle_mesh("MiddleLegLinkage.obj")

# 将每个顶点的颜色设置为深灰色
num_vertices = len(mesh.vertices)
mesh.vertex_colors = o3d.utility.Vector3dVector([[0.2, 0.2, 0.2] for _ in range(num_vertices)])

# 创建渲染窗口
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(mesh)

# 设置相机参数
ctr = vis.get_view_control()
ctr.change_field_of_view(step=-90)  # 改变视角，确保模型在视野内

# 运行渲染
vis.run()
vis.destroy_window()
