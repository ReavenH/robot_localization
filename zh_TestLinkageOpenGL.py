import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GL.shaders import compileProgram, compileShader
import numpy as np
import pyassimp.pyassimp

# Load model using Assimp
def load_model(file_path):
    scene = pyassimp.load(file_path)
    if not scene.meshes:
        raise Exception("No meshes found in file")
    mesh = scene.meshes[0]
    vertices = np.array(mesh.vertices, dtype=np.float32)
    pyassimp.release(scene)
    return vertices

# Initialize OpenGL
def init_opengl():
    glEnable(GL_DEPTH_TEST)
    glClearColor(0.1, 0.1, 0.1, 1)

# Load and compile shaders
def load_shader(shader_file, shader_type):
    with open(shader_file, 'r') as f:
        shader_source = f.read()
    shader = compileShader(shader_source, shader_type)
    return shader

# Main function
def main():
    # Initialize pygame
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

    # Initialize OpenGL
    init_opengl()

    # Load shaders
    vertex_shader_source = """
    #version 330 core
    layout(location = 0) in vec3 position;

    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;

    void main()
    {
        gl_Position = projection * view * model * vec4(position, 1.0);
    }
    """

    fragment_shader_source = """
    #version 330 core
    out vec4 FragColor;

    void main()
    {
        FragColor = vec4(1.0, 1.0, 1.0, 1.0); // White color
    }
    """

    vertex_shader = compileShader(vertex_shader_source, GL_VERTEX_SHADER)
    fragment_shader = compileShader(fragment_shader_source, GL_FRAGMENT_SHADER)
    shader_program = compileProgram(vertex_shader, fragment_shader)

    # Load model
    vertices = load_model('zh_OBJTest.obj')

    # Setup buffers
    VAO = glGenVertexArrays(1)
    VBO = glGenBuffers(1)

    glBindVertexArray(VAO)

    glBindBuffer(GL_ARRAY_BUFFER, VBO)
    glBufferData(GL_ARRAY_BUFFER, vertices.nbytes, vertices, GL_STATIC_DRAW)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * vertices.itemsize, ctypes.c_void_p(0))
    glEnableVertexAttribArray(0)

    glBindBuffer(GL_ARRAY_BUFFER, 0)
    glBindVertexArray(0)

    # Projection matrix
    projection = np.identity(4, dtype=np.float32)
    projection[3][3] = 1.0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glUseProgram(shader_program)

        view = np.identity(4, dtype=np.float32)
        view[3][2] = -3.0  # Move back to view the object

        model = np.identity(4, dtype=np.float32)
        model = np.dot(model, np.array([
            [np.cos(pygame.time.get_ticks() / 1000), 0, np.sin(pygame.time.get_ticks() / 1000), 0],
            [0, 1, 0, 0],
            [-np.sin(pygame.time.get_ticks() / 1000), 0, np.cos(pygame.time.get_ticks() / 1000), 0],
            [0, 0, 0, 1]
        ], dtype=np.float32))

        model_loc = glGetUniformLocation(shader_program, "model")
        view_loc = glGetUniformLocation(shader_program, "view")
        projection_loc = glGetUniformLocation(shader_program, "projection")

        glUniformMatrix4fv(model_loc, 1, GL_FALSE, model)
        glUniformMatrix4fv(view_loc, 1, GL_FALSE, view)
        glUniformMatrix4fv(projection_loc, 1, GL_FALSE, projection)

        glBindVertexArray(VAO)
        glDrawArrays(GL_TRIANGLES, 0, len(vertices) // 3)

        pygame.display.flip()
        pygame.time.wait(10)

    glDeleteVertexArrays(1, VAO)
    glDeleteBuffers(1, VBO)
    glDeleteProgram(shader_program)
    pygame.quit()

if __name__ == "__main__":
    main()
