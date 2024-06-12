import open3d as o3d
import numpy as np
import glfw
from OpenGL.GL import *
from OpenGL.GL.shaders import compileProgram, compileShader
from pyrr import Matrix44

# 顶点着色器
vertex_src = """
# version 330 core
layout(location = 0) in vec3 a_position;
uniform mat4 u_transform;
void main()
{
    gl_Position = u_transform * vec4(a_position, 1.0);
}
"""

# 片段着色器
fragment_src = """
# version 330 core
out vec4 out_color;
void main()
{
    out_color = vec4(0.8, 0.8, 0.2, 1.0); // 黄色
}
"""

# 初始化OpenGL
def initialize_opengl():
    # 初始化GLFW
    if not glfw.init():
        return

    # 创建窗口
    window = glfw.create_window(800, 600, "OpenGL Example", None, None)
    if not window:
        glfw.terminate()
        return

    # 设置窗口为当前上下文
    glfw.make_context_current(window)

    # 编译着色器
    shader = compileProgram(
        compileShader(vertex_src, GL_VERTEX_SHADER),
        compileShader(fragment_src, GL_FRAGMENT_SHADER)
    )

    # 使用着色器程序
    glUseProgram(shader)

    # 开启深度测试
    glEnable(GL_DEPTH_TEST)

    return window, shader

# 加载OBJ文件并获取顶点数据
def load_obj(filename):
    mesh = o3d.io.read_triangle_mesh(filename)
    vertices = np.asarray(mesh.vertices)
    return vertices

# 主循环
def main():
    # 初始化OpenGL
    window, shader = initialize_opengl()

    # 加载OBJ文件
    vertices = load_obj("MiddleLegLinkage.obj")

    # 创建缓冲区
    vbo = glGenBuffers(1)
    glBindBuffer(GL_ARRAY_BUFFER, vbo)
    glBufferData(GL_ARRAY_BUFFER, vertices.nbytes, vertices, GL_STATIC_DRAW)

    # 获取顶点位置的位置
    position = glGetAttribLocation(shader, "a_position")
    glEnableVertexAttribArray(position)
    glVertexAttribPointer(position, 3, GL_FLOAT, GL_FALSE, 0, None)

    # 创建变换矩阵
    transform = glGetUniformLocation(shader, "u_transform")
    view = Matrix44.from_eulers((0.0, 0.0, 0.0), dtype=np.float32)
    projection = Matrix44.perspective_projection(45.0, 800 / 600, 0.1, 100.0, dtype=np.float32)
    model = Matrix44.from_translation([0.0, 0.0, -2.0], dtype=np.float32)

    # 主循环
    while not glfw.window_should_close(window):
        # 清空颜色缓冲区和深度缓冲区
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # 更新变换矩阵
        glUniformMatrix4fv(transform, 1, GL_FALSE, (projection * view * model).flatten())

        # 绘制模型
        glDrawArrays(GL_TRIANGLES, 0, len(vertices))

        # 交换缓冲区和检查事件
        glfw.swap_buffers(window)
        glfw.poll_events()

    # 清理资源
    glfw.terminate()

if __name__ == "__main__":
    main()
