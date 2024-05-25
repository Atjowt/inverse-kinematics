#include "cglm/vec2.h"
#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <cglm/cglm.h>
#include <stdlib.h>
#include <stdio.h>

#define ARM_LENGTH 4

typedef struct {
    vec2 position;
    float length;
    float angle;
} Segment;

void error_callback(int error, const char* description);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos);
void window_point_to_device_point(GLFWwindow* window, double x, double y, double* out_x, double* out_y);
void init_arm(Segment arm[ARM_LENGTH]);
void inverse_kinematics(Segment* segments, int n, vec2 target);
void update_segment_positions(Segment* segments, int n);

int render_width, render_height;
double mouse_x, mouse_y;

int main(void) {
    const int init_window_width = 800;
    const int init_window_height = 500;
    const int n_x_MSAA = 4; // Anti-aliasing

    if (!glfwInit()) {
        fprintf(stderr, "Error initializing GLFW");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_FALSE);
    glfwWindowHint(GLFW_SAMPLES, n_x_MSAA);

    GLFWwindow* window = glfwCreateWindow(init_window_width, init_window_height, "Inverse Kinematics", NULL, NULL);
    if (!window) {
        fprintf(stderr, "Error creating window");
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetKeyCallback(window, key_callback);
    glfwSetErrorCallback(error_callback);
    glfwSetCursorPosCallback(window, cursor_pos_callback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        fprintf(stderr, "Error loading GLAD loader");
        glfwTerminate();
        return 1;
    }

    glfwSwapInterval(1);
    framebuffer_size_callback(window, init_window_width, init_window_height);

    Segment arm[ARM_LENGTH];
    init_arm(arm);

    while (!glfwWindowShouldClose(window)) {
        double target_x, target_y;
        window_point_to_device_point(window, mouse_x, mouse_y, &target_x, &target_y);
        vec2 target = { target_x, target_y };
        inverse_kinematics(arm, ARM_LENGTH, target);

        glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glLineWidth(16.0f);
        glEnable(GL_LINE_SMOOTH);
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

        glBegin(GL_LINES);
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        for (int i = 0; i < ARM_LENGTH - 1; i++) {
            glVertex2f(arm[i].position[0], arm[i].position[1]);
            glVertex2f(arm[i + 1].position[0], arm[i + 1].position[1]);
        }
        glEnd();

        glDisable(GL_LINE_SMOOTH);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);

    return 0;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    render_width = width;
    render_height = height;
    printf("Resized to [%d x %d]\n", width, height);
    glViewport(0, 0, width, height);
}

void error_callback(int error, const char* description) {
    fprintf(stderr, "Error: %s\n", description);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        printf("Closing window...\n");
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos) {
    printf("Mouse pos [%d, %d]\n", (int)xpos, (int)ypos);
    mouse_x = xpos;
    mouse_y = ypos;
}

void window_point_to_device_point(GLFWwindow* window, double x, double y, double* out_x, double* out_y) {
    *out_x = (x / render_width) * 2.0 - 1.0;
    *out_y = 1.0 - (y / render_height) * 2.0;
}

void init_arm(Segment arm[ARM_LENGTH]) {
    static const float segment_length = 0.25f;
    for (int i = 0; i < ARM_LENGTH; i++) {
        arm[i].length = segment_length;
        arm[i].angle = 0.0f;
        arm[i].position[0] = (i == 0) ? 0.0f : arm[i - 1].position[0] + arm[i - 1].length;
        arm[i].position[1] = 0.0f;
    }
}

void inverse_kinematics(Segment* segments, int n, vec2 target) {

    static const int max_iterations = 12;
    static const float target_threshold = 0.01f;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        for (int i = n - 1; i >= 0; --i) {
            // Compute the vector from the current joint to the end effector and to the target
            vec2 to_end_effector;
            glm_vec2_sub(segments[n - 1].position, segments[i].position, to_end_effector);
            vec2 to_target;
            glm_vec2_sub(target, segments[i].position, to_target);

            // Compute the angles of these vectors
            float angle_to_end_effector = atan2f(to_end_effector[1], to_end_effector[0]);
            float angle_to_target = atan2f(to_target[1], to_target[0]);

            // Compute the rotation needed to align the end effector with the target
            float rotation = angle_to_target - angle_to_end_effector;

            // Rotate the segment
            segments[i].angle += rotation;

            // Update the positions of all segments
            update_segment_positions(segments, n);

            if (glm_vec2_distance(segments[n - 1].position, target) < target_threshold) {
                return;
            }
        }
    }
}

void update_segment_positions(Segment* segments, int n) {
    for (int i = 0; i < n; ++i) {
        if (i == 0) {
            glm_vec2_zero(segments[i].position);
        } else {
            vec2 v = {
                cosf(segments[i - 1].angle) * segments[i - 1].length,
                sinf(segments[i - 1].angle) * segments[i - 1].length,
            };
            glm_vec2_add(segments[i - 1].position, v, segments[i].position);
        }
    }
}
