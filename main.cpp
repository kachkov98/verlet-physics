#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <cmath>

#include "bindings/imgui_impl_glfw.h"
#include "bindings/imgui_impl_opengl3.h"

#include "physics.hpp"

void generateStaticBox(World &world, glm::vec2 pos, float width, float height) {
  world.addCollider({
    world.addPoint(glm::vec2(pos.x - width / 2, pos.y + height / 2), PointIterator::STATIC),
    world.addPoint(glm::vec2(pos.x + width / 2, pos.y + height / 2), PointIterator::STATIC),
    world.addPoint(glm::vec2(pos.x + width / 2, pos.y - height / 2), PointIterator::STATIC),
    world.addPoint(glm::vec2(pos.x - width / 2, pos.y - height / 2), PointIterator::STATIC)
  });
}

void generateDynamicBox(World &world, glm::vec2 pos, float size) {
  std::vector<PointIterator> points = {
    world.addPoint(glm::vec2(pos.x - size / 2, pos.y + size / 2), PointIterator::DYNAMIC),
    world.addPoint(glm::vec2(pos.x + size / 2, pos.y + size / 2), PointIterator::DYNAMIC),
    world.addPoint(glm::vec2(pos.x + size / 2, pos.y - size / 2), PointIterator::DYNAMIC),
    world.addPoint(glm::vec2(pos.x - size / 2, pos.y - size / 2), PointIterator::DYNAMIC)
  };
  world.addCollider(points);
  world.addLink({points[0], points[1]});
  world.addLink({points[1], points[2]});
  world.addLink({points[2], points[3]});
  world.addLink({points[3], points[0]});
  world.addLink({points[0], points[2]});
  world.addLink({points[1], points[3]});
}

void generatePolygon(World &world, glm::vec2 pos, unsigned n, float radius) {
  std::vector<PointIterator> points;
  points.reserve(n);
  for (unsigned i = 0; i < n; ++i) {
    float angle = 2.0f * glm::pi<float>() * i / n;
    glm::vec2 p = pos + radius * glm::vec2(sin(angle), cos(angle));
    points.push_back(world.addPoint(p, PointIterator::DYNAMIC));
  }
  for (unsigned i = 0; i < n; ++i)
    for (unsigned j = i + 1; j < n; ++j)
      world.addLink({points[i], points[j]});
  world.addCollider(points);
}

void generateRope(World &world, glm::vec2 start, glm::vec2 segment, unsigned n) {
  PointIterator prevPoint = world.addPoint(start, PointIterator::STATIC);
  for (unsigned i = 0; i < n; ++i) {
    PointIterator curPoint = world.addPoint(world.getPoint(prevPoint).get() + segment,
                                            PointIterator::DYNAMIC);
    world.addLink({prevPoint, curPoint});
    prevPoint = curPoint;
  }
  glm::vec2 p0 = world.getPoint(prevPoint).get();
  PointIterator p1 = world.addPoint(p0 + glm::vec2( 0.5f, -1.0f),
                                    PointIterator::DYNAMIC);
  PointIterator p2 = world.addPoint(p0 + glm::vec2(-0.5f, -1.0f),
                                    PointIterator::DYNAMIC);
  world.addCollider({prevPoint, p1, p2});
  world.addLink({prevPoint, p1});
  world.addLink({prevPoint, p2});
  world.addLink({p1, p2});
}

void generateScene(World &world) {
  generateStaticBox(world, glm::vec2(0.0f, -1.5f), 30.0f, 1.0f);
  generateStaticBox(world, glm::vec2(-15.5f, 5.0f), 1.0f, 14.0f);
  generateStaticBox(world, glm::vec2( 15.5f, 5.0f), 1.0f, 14.0f);
  for (int x = -10; x <= 10; x += 2)
    for (int y = 0; y <= 4; y += 2)
    generateDynamicBox(world, glm::vec2((float)x, (float)y), 1.0f);
  for (unsigned i = 3; i <= 7; ++i)
    generatePolygon(world, glm::vec2((float)i * 2.0f  - 10.0f, 8.0f), i, 1.0f);
  generateRope(world, glm::vec2(0.0f, 10.0f), glm::vec2(0.0f, -0.25f), 20);
}

int main(int argc, char *argv[]) {
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  GLFWwindow* window = glfwCreateWindow(1280, 720, "Physics demo", NULL, NULL);
  if (window == NULL) {
    printf("Failed to create GLFW window\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    printf("Failed to initialize GLAD\n");
    glfwTerminate();
    return 1;
  }
  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  // Setup Platform/Renderer bindings
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init();
  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // Setup engine
  World world;
  generateScene(world);
  Viewport viewport(ImGui::GetMainViewport()->Size, glm::vec2(0.0f, 5.0f), 40.0f);
  std::optional<PointIterator> draggedPoint;
  while(!glfwWindowShouldClose(window)) {
    glClearColor(0.0f, 0.0f, 0.0f, 1.00f);
    glClear(GL_COLOR_BUFFER_BIT);

    // feed inputs to dear imgui, start new frame
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // render to dear imgui
    viewport.setExtent(ImGui::GetMainViewport()->Size);
    if (ImGui::IsMouseDragging(1)) {
      viewport.moveCenter(ImGui::GetMouseDragDelta(1));
      ImGui::ResetMouseDragDelta(1);
    }
    viewport.changeScale(std::pow(1.25f, ImGui::GetIO().MouseWheel));
    if (ImGui::IsMouseClicked(0)) {
      glm::vec2 mousePos = viewport.screenToWorld(ImGui::GetMousePos());
      draggedPoint = world.getNearestDynamicPoint(mousePos, 0.2f);
    }
    if (ImGui::IsMouseReleased(0))
      draggedPoint = std::nullopt;
    if (draggedPoint) {
      glm::vec2 mousePos = viewport.screenToWorld(ImGui::GetMousePos());
      world.setPoint(*draggedPoint, mousePos);
    }
    world.update(10);
    world.render(viewport);

    // render dear imgui into screen
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwTerminate();
  return 0;
}
