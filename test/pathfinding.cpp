#include "behaviorBuilder.h"
#include "pathfinder.h"
#include "triangle_lib_navmesh.h"
#include "vector.h"

#include "triangle/triangle_api.h"

#include "math_helpers.h"

#include <absl/log/globals.h>
#include <absl/strings/string_view.h>

#include <gtest/gtest.h>

#include <filesystem>

class PolyFileTestFixture : public ::testing::Test {
protected:
  PolyFileTestFixture() {
    // Turn off debug logging so the Pathfinder doesnt print.
    absl::SetGlobalVLogLevel(0);
  }

  void loadPolyFile(absl::string_view filename) {

    // Initialize triangle data
    triangle::triangleio triangleData;
    triangle::triangleio triangleVoronoiData;
    triangle::triangle_initialize_triangleio(&triangleData);
    triangle::triangle_initialize_triangleio(&triangleVoronoiData);
    initTriangleData(filename, triangleData, triangleVoronoiData);

    // Build navmesh from triangle data
    navmesh_ = pathfinder::navmesh::TriangleLibNavmesh(triangleData, triangleVoronoiData);

    // Free triangle data
    triangle_free_triangleio(&triangleData);
    triangle_free_triangleio(&triangleVoronoiData);
  }

  std::optional<pathfinder::navmesh::TriangleLibNavmesh> navmesh_;
  pathfinder::BehaviorBuilder behaviorBuilder_;

private:
  void initTriangleData(absl::string_view filename, triangle::triangleio &triangleData, triangle::triangleio &triangleVoronoiData) {
    
    triangle::triangleio inputData;
    triangle::triangle_initialize_triangleio(&inputData);

    int firstNode;
    int readFileResult = triangle::triangle_read_poly(filename.data(), &inputData, &firstNode);
    if (readFileResult < 0) {
      throw std::runtime_error("Unable to open .poly file, error "+std::to_string(readFileResult)+". Current path is "+std::filesystem::current_path().string());
    }

    // Create a context
    triangle::context *ctx;
    ctx = triangle::triangle_context_create();

    // Set context's behavior
    triangle::behavior_t behavior = behaviorBuilder_.getBehavior();
    int behaviorSetResult = triangle::triangle_context_set_behavior(ctx, &behavior);
    if (behaviorSetResult < 0) {
      // Free memory
      triangle_context_destroy(ctx);
      triangle_free_triangleio(&inputData);
      throw std::runtime_error("Error setting behavior ("+std::to_string(behaviorSetResult)+")");
    }

    // Build the triangle mesh
    int meshCreateResult = triangle::triangle_mesh_create(ctx, &inputData);
    if (meshCreateResult < 0) {
      // Free memory
      triangle_context_destroy(ctx);
      triangle_free_triangleio(&inputData);
      throw std::runtime_error("Error creating navmesh ("+std::to_string(meshCreateResult)+")");
    }

    // Now, the context holds the mesh, lets extract this data
    // Copy data
    int copyResult = triangle::triangle_mesh_copy(ctx, &triangleData, 1, 1, &triangleVoronoiData);
    if (copyResult < 0) {
      // Free memory
      triangle_context_destroy(ctx);
      triangle_free_triangleio(&inputData);
      throw std::runtime_error("Error copying navmesh data ("+std::to_string(copyResult)+")");
    }

    // Done with input data
    triangle_free_triangleio(&inputData);

    // Done with context
    triangle_context_destroy(ctx);
  }
};

// ===================================================================================================================
// ============================================= startTouchingVertexTest =============================================
// ===================================================================================================================

TEST_F(PolyFileTestFixture, StartTouchingVertexTest_StartTopGoingLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/startTouchingVertexTest.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{500.0, 750.0-kAgentRadius};
  const pathfinder::Vector goalPoint{400.0, 750.0-kAgentRadius};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_EQ(pathLength, 100.0);
}

TEST_F(PolyFileTestFixture, StartTouchingVertexTest_StartTopGoingRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/startTouchingVertexTest.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{500.0, 750.0-kAgentRadius};
  const pathfinder::Vector goalPoint{600.0, 750.0-kAgentRadius};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_EQ(pathLength, 100.0);
}

TEST_F(PolyFileTestFixture, StartTouchingVertexTest_StartBottomGoingLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/startTouchingVertexTest.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{500.0, 250.0+kAgentRadius};
  const pathfinder::Vector goalPoint{400.0, 250.0+kAgentRadius};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_EQ(pathLength, 100.0);
}

TEST_F(PolyFileTestFixture, StartTouchingVertexTest_StartBottomGoingRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/startTouchingVertexTest.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{500.0, 250.0+kAgentRadius};
  const pathfinder::Vector goalPoint{600.0, 250.0+kAgentRadius};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_EQ(pathLength, 100.0);
}

// Misc test cases using this navmesh:

TEST_F(PolyFileTestFixture, StartTouchingVertexTest_NotTouchingVertexBut1) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/startTouchingVertexTest.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{165.0, 165.0};
  const pathfinder::Vector goalPoint{400.0, 250.0+kAgentRadius};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 278.47832, 1e-5);
}

TEST_F(PolyFileTestFixture, StartTouchingVertexTest_NotTouchingVertexBut2) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/startTouchingVertexTest.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{835.0,165.0};
  const pathfinder::Vector goalPoint{600.0, 250.0+kAgentRadius};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 278.47832, 1e-5);
}

TEST_F(PolyFileTestFixture, StartTouchingVertexTest_NotTouchingVertexBut3) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/startTouchingVertexTest.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{165.0,835.0};
  const pathfinder::Vector goalPoint{400.0, 750.0-kAgentRadius};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 278.47832, 1e-5);
}

TEST_F(PolyFileTestFixture, StartTouchingVertexTest_NotTouchingVertexBut4) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/startTouchingVertexTest.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{835.0,835.0};
  const pathfinder::Vector goalPoint{600.0, 750.0-kAgentRadius};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 278.47832, 1e-5);
}

// ===================================================================================================================
// ================================================= exactFitIonTrap =================================================
// ===================================================================================================================

TEST_F(PolyFileTestFixture, ExactFitIonTrap_StartBottomLeftGoingBottomRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitIonTrap.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{15.0, 30.0};
  const pathfinder::Vector goalPoint{85.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

TEST_F(PolyFileTestFixture, ExactFitIonTrap_StartBottomLeftGoingTopRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitIonTrap.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{15.0, 30.0};
  const pathfinder::Vector goalPoint{85.0, 70.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

TEST_F(PolyFileTestFixture, ExactFitIonTrap_StartTopLeftGoingBottomRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitIonTrap.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{15.0, 70.0};
  const pathfinder::Vector goalPoint{85.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

TEST_F(PolyFileTestFixture, ExactFitIonTrap_StartTopLeftGoingTopRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitIonTrap.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{15.0, 70.0};
  const pathfinder::Vector goalPoint{85.0, 70.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

TEST_F(PolyFileTestFixture, ExactFitIonTrap_StartBottomRightGoingBottomLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitIonTrap.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{85.0, 30.0};
  const pathfinder::Vector goalPoint{15.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

TEST_F(PolyFileTestFixture, ExactFitIonTrap_StartBottomRightGoingTopLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitIonTrap.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{85.0, 30.0};
  const pathfinder::Vector goalPoint{15.0, 70.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

TEST_F(PolyFileTestFixture, ExactFitIonTrap_StartTopRightGoingBottomLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitIonTrap.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{85.0, 70.0};
  const pathfinder::Vector goalPoint{15.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

TEST_F(PolyFileTestFixture, ExactFitIonTrap_StartTopRightGoingTopLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitIonTrap.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{85.0, 70.0};
  const pathfinder::Vector goalPoint{15.0, 70.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

// ===================================================================================================================
// ================================================ exactFitCorridors ================================================
// ===================================================================================================================

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartLeftTopGoingLeftBottom) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{30.0, 140.0};
  const pathfinder::Vector goalPoint{30.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartLeftTopGoingRightTop) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{30.0, 140.0};
  const pathfinder::Vector goalPoint{170.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartLeftTopGoingRightBottom) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{30.0, 140.0};
  const pathfinder::Vector goalPoint{170.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartLeftTopGoingBottomLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{30.0, 140.0};
  const pathfinder::Vector goalPoint{60.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartLeftTopGoingBottomRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{30.0, 140.0};
  const pathfinder::Vector goalPoint{140.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartLeftTopGoingTopLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{30.0, 140.0};
  const pathfinder::Vector goalPoint{60.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartLeftTopGoingTopRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{30.0, 140.0};
  const pathfinder::Vector goalPoint{140.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartLeftBottomGoingLeftTop) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{30.0, 60.0};
  const pathfinder::Vector goalPoint{30.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartLeftBottomGoingRightTop) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{30.0, 60.0};
  const pathfinder::Vector goalPoint{170.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartLeftBottomGoingRightBottom) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{30.0, 60.0};
  const pathfinder::Vector goalPoint{170.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartLeftBottomGoingBottomLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{30.0, 60.0};
  const pathfinder::Vector goalPoint{60.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartLeftBottomGoingBottomRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{30.0, 60.0};
  const pathfinder::Vector goalPoint{140.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartLeftBottomGoingTopLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{30.0, 60.0};
  const pathfinder::Vector goalPoint{60.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartLeftBottomGoingTopRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{30.0, 60.0};
  const pathfinder::Vector goalPoint{140.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartRightTopGoingLeftTop) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{170.0, 140.0};
  const pathfinder::Vector goalPoint{30.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartRightTopGoingLeftBottom) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{170.0, 140.0};
  const pathfinder::Vector goalPoint{30.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartRightTopGoingRightBottom) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{170.0, 140.0};
  const pathfinder::Vector goalPoint{170.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartRightTopGoingBottomLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{170.0, 140.0};
  const pathfinder::Vector goalPoint{60.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartRightTopGoingBottomRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{170.0, 140.0};
  const pathfinder::Vector goalPoint{140.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartRightTopGoingTopLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{170.0, 140.0};
  const pathfinder::Vector goalPoint{60.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartRightTopGoingTopRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{170.0, 140.0};
  const pathfinder::Vector goalPoint{140.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartRightBottomGoingLeftTop) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{170.0, 60.0};
  const pathfinder::Vector goalPoint{30.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartRightBottomGoingLeftBottom) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{170.0, 60.0};
  const pathfinder::Vector goalPoint{30.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartRightBottomGoingRightTop) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{170.0, 60.0};
  const pathfinder::Vector goalPoint{170.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartRightBottomGoingBottomLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{170.0, 60.0};
  const pathfinder::Vector goalPoint{60.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartRightBottomGoingBottomRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{170.0, 60.0};
  const pathfinder::Vector goalPoint{140.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartRightBottomGoingTopLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{170.0, 60.0};
  const pathfinder::Vector goalPoint{60.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartRightBottomGoingTopRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{170.0, 60.0};
  const pathfinder::Vector goalPoint{140.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartBottomLeftGoingLeftTop) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{60.0, 30.0};
  const pathfinder::Vector goalPoint{30.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartBottomLeftGoingLeftBottom) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{60.0, 30.0};
  const pathfinder::Vector goalPoint{30.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartBottomLeftGoingRightTop) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{60.0, 30.0};
  const pathfinder::Vector goalPoint{170.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartBottomLeftGoingRightBottom) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{60.0, 30.0};
  const pathfinder::Vector goalPoint{170.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartBottomLeftGoingBottomRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{60.0, 30.0};
  const pathfinder::Vector goalPoint{140.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartBottomLeftGoingTopLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{60.0, 30.0};
  const pathfinder::Vector goalPoint{60.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartBottomLeftGoingTopRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{60.0, 30.0};
  const pathfinder::Vector goalPoint{140.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartBottomRightGoingLeftTop) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{140.0, 30.0};
  const pathfinder::Vector goalPoint{30.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartBottomRightGoingLeftBottom) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{140.0, 30.0};
  const pathfinder::Vector goalPoint{30.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartBottomRightGoingRightTop) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{140.0, 30.0};
  const pathfinder::Vector goalPoint{170.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartBottomRightGoingRightBottom) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{140.0, 30.0};
  const pathfinder::Vector goalPoint{170.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartBottomRightGoingBottomLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{140.0, 30.0};
  const pathfinder::Vector goalPoint{60.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartBottomRightGoingTopLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{140.0, 30.0};
  const pathfinder::Vector goalPoint{60.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartBottomRightGoingTopRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{140.0, 30.0};
  const pathfinder::Vector goalPoint{140.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartTopLeftGoingLeftTop) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{60.0, 170.0};
  const pathfinder::Vector goalPoint{30.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartTopLeftGoingLeftBottom) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{60.0, 170.0};
  const pathfinder::Vector goalPoint{30.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartTopLeftGoingRightTop) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{60.0, 170.0};
  const pathfinder::Vector goalPoint{170.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartTopLeftGoingRightBottom) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{60.0, 170.0};
  const pathfinder::Vector goalPoint{170.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartTopLeftGoingBottomLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{60.0, 170.0};
  const pathfinder::Vector goalPoint{60.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartTopLeftGoingBottomRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{60.0, 170.0};
  const pathfinder::Vector goalPoint{140.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartTopLeftGoingTopRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{60.0, 170.0};
  const pathfinder::Vector goalPoint{140.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartTopRightGoingLeftTop) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{140.0, 170.0};
  const pathfinder::Vector goalPoint{30.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartTopRightGoingLeftBottom) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{140.0, 170.0};
  const pathfinder::Vector goalPoint{30.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartTopRightGoingRightTop) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{140.0, 170.0};
  const pathfinder::Vector goalPoint{170.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartTopRightGoingRightBottom) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{140.0, 170.0};
  const pathfinder::Vector goalPoint{170.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartTopRightGoingBottomLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{140.0, 170.0};
  const pathfinder::Vector goalPoint{60.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartTopRightGoingBottomRight) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{140.0, 170.0};
  const pathfinder::Vector goalPoint{140.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(PolyFileTestFixture, ExactFitCorridors_StartTopRightGoingTopLeft) {
  constexpr const double kAgentRadius{10.0};
  constexpr const absl::string_view kPolyFilename{"poly/exactFitCorridors.poly"};
  loadPolyFile(kPolyFilename);

  const pathfinder::Vector startPoint{140.0, 170.0};
  const pathfinder::Vector goalPoint{60.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(kAgentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}

// ===================================================================================================================
// =============================================Parameterized test classes============================================
// ===================================================================================================================

using StraightPathParams = std::tuple<
    /*Poly filename=*/const char*,
    /*Start=*/pathfinder::Vector,
    /*Goal=*/pathfinder::Vector,
    /*Agent Radius=*/double
>;

using NonStraightPathParams = std::tuple<
    /*Poly filename=*/const char*,
    /*Start=*/pathfinder::Vector,
    /*Goal=*/pathfinder::Vector,
    /*Agent Radius=*/double,
    /*Expected Path Length=*/double,
    /*Allowed Precision Tolerance=*/double
>;

using NoPathParams = std::tuple<
    /*Poly filename=*/const char*,
    /*Start=*/pathfinder::Vector,
    /*Goal=*/pathfinder::Vector,
    /*Agent Radius=*/double
>;

class ParamStraightPaths : public PolyFileTestFixture, public testing::WithParamInterface<StraightPathParams> {};
class ParamNonStraightPaths : public PolyFileTestFixture, public testing::WithParamInterface<NonStraightPathParams> {};
class ParamNoPaths : public PolyFileTestFixture, public testing::WithParamInterface<NoPathParams> {};

TEST_P(ParamStraightPaths, TestName) {
  const auto &param = GetParam();
  loadPolyFile(std::get<0>(param));
  const pathfinder::Vector &startPoint = std::get<1>(param);
  const pathfinder::Vector &goalPoint = std::get<2>(param);
  const double agentRadius = std::get<3>(param);
  const double expectedLength = pathfinder::math::distance(startPoint, goalPoint);

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(agentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  {
    // Check start->goal
    const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);
    const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
    EXPECT_EQ(pathLength, expectedLength);
  }
  {
    // Check goal->start
    const auto pathfindingResult = pathfinder.findShortestPath(goalPoint, startPoint);
    const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
    EXPECT_EQ(pathLength, expectedLength);
  }
}

TEST_P(ParamNonStraightPaths, TestName) {
  const auto &param = GetParam();
  loadPolyFile(std::get<0>(param));
  const pathfinder::Vector &startPoint = std::get<1>(param);
  const pathfinder::Vector &goalPoint = std::get<2>(param);
  const double agentRadius = std::get<3>(param);
  const double expectedLength = std::get<4>(param);
  const double precisionTolerance = std::get<5>(param);

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(agentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  {
    // Check start->goal
    const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);
    const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
    EXPECT_NEAR(pathLength, expectedLength, precisionTolerance);
  }
  {
    // Check goal->start
    const auto pathfindingResult = pathfinder.findShortestPath(goalPoint, startPoint);
    const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
    EXPECT_NEAR(pathLength, expectedLength, precisionTolerance);
  }
}

TEST_P(ParamNoPaths, TestName) {
  const auto &param = GetParam();
  loadPolyFile(std::get<0>(param));
  const pathfinder::Vector &startPoint = std::get<1>(param);
  const pathfinder::Vector &goalPoint = std::get<2>(param);
  const double agentRadius = std::get<3>(param);

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::PathfinderConfig config;
  config.setAgentRadius(agentRadius);
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, config);
  {
    // Check start->goal
    const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);
    EXPECT_TRUE(pathfindingResult.shortestPath.empty());
  }
  {
    // Check goal->start
    const auto pathfindingResult = pathfinder.findShortestPath(goalPoint, startPoint);
    EXPECT_TRUE(pathfindingResult.shortestPath.empty());
  }
}

INSTANTIATE_TEST_SUITE_P(StartTouchingVertexTestSuite,
                         ParamStraightPaths,
                         testing::Values(
    StraightPathParams{"poly/startTouchingVertexTest.poly", {500.00000000000005684342, 458.33333333333337122895}, {640.5, 530.0}, 3.0 },
    StraightPathParams{"poly/startTouchingVertexTest.poly",  {509.0000000000, 406.0000000000}, {496.0000000000, 713.0000000000}, 10.0000000000 }
));

INSTANTIATE_TEST_SUITE_P(StartTouchingVertexTestSuite,
                         ParamNonStraightPaths,
                         testing::Values(
    // A bug in math::angle when `dx` was negative and extremely close to 0. It happens with other radii.
    NonStraightPathParams{"poly/startTouchingVertexTest.poly", {500.00000000000005684342, 458.33333333333337122895}, {218.0, 882.0}, 3.0, 521.4283184304, 1e-10 },

    // A bug in the calculation of a temporary left interval due to the reduced precision used in math::createCircleConsciousLine.
    NonStraightPathParams{"poly/startTouchingVertexTest.poly", {746.0, 582.0}, {186.744, 907.560}, 3.0, 696.08533, 1e-5 },
    NonStraightPathParams{"poly/startTouchingVertexTest.poly", {880.0, 538.0}, {186.744, 907.560}, 3.0, 837.12361, 1e-5 },
    NonStraightPathParams{"poly/startTouchingVertexTest.poly", {560.759, 645.189}, {186.744, 907.560}, 3.0, 500.37311, 1e-5 },

    // This error uncovered a bug in the calculation of the estimated distance to the goal. It happens with other radii.
    NonStraightPathParams{"poly/startTouchingVertexTest.poly", {125.0, 125.0}, {500.0, 500.0}, 10.0, 530.75444, 1e-5 },

    NonStraightPathParams{"poly/startTouchingVertexTest.poly", {563.32619,284.91002}, {218.0,86.0}, 3.0, 486.2003865074, 1e-10 },
    NonStraightPathParams{"poly/startTouchingVertexTest.poly", {592.29351,298.74423}, {218.0, 86.0}, 3.0, 516.5894574398, 1e-12 },
    NonStraightPathParams{"poly/startTouchingVertexTest.poly", {808.85775, 252.71609}, {218.0, 86.0}, 3.0, 730.1124419612, 1e-10 },
    NonStraightPathParams{"poly/startTouchingVertexTest.poly", {650.50155,747.56657}, {194.62174,943.38572}, 3.0, 605.559296966, 1e-11 }
));

INSTANTIATE_TEST_SUITE_P(ExactFitCorridorsSuite,
                         ParamNonStraightPaths,
                         testing::Values(
    NonStraightPathParams{"poly/exactFitCorridors.poly", {68.7320000000, 122.2710000000}, {119.0070000000, 78.3080000000}, 1.0000000000, 377.8150290897, 1e-9 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {68.7320000000, 122.2710000000}, {11.4935337311, 180.1627053697}, 1.0000000000, 81.4107369828, 1e-11 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {54.2593301530, 136.8528148457}, {11.5683466701, 180.2545212494}, 1.00000, 60.878908123, 1e-11 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {141.0219359028, 132.4184479139}, {2.5471496995, 198.0745867794}, 1.0000000000, 199.3466177188, 1e-10 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {191.3710438722, 147.8571180599}, {2.5471496995, 198.0745867794}, 1.0000000000, 201.4981146012, 1e-11 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {195.1427287939, 146.9394554285}, {2.5471496995, 198.0745867794}, 1.0000000000, 205.0330827951, 1e-11 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {45.0464956919, 154.9535043081}, {67.2500000000, 142.7500000000}, 7.0710000000, 25.3615695556, 1e-10 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {118.1359777349, 117.7007024532}, {157.0081410555, 157.2923379117}, 7.0710000000, 55.490200365937838000718329567462, 2e-14 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {82.29929754680,118.1359777349}, {42.70766208830,157.0081410555}, 7.0710000000, 55.490200365937838000718329567462, 2e-14 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {81.86402226510,82.29929754680}, {42.99185894450,42.70766208830}, 7.0710000000, 55.490200365937838000718329567462, 2e-14 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {117.7007024532,81.86402226510}, {157.2923379117,42.99185894450}, 7.0710000000, 55.490200365937838000718329567462, 2e-14 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {118.1359777349, 117.7007024532}, {167.2268895566, 172.1750442220}, 7.0710000000, 73.53064455046415, 5e-12 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {82.29929754680,118.1359777349}, {27.82495577800,167.2268895566}, 7.0710000000, 73.53064455046415, 5e-12 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {81.86402226510,82.29929754680}, {32.77311044340,27.82495577800}, 7.0710000000, 73.53064455046415, 5e-12 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {117.7007024532,81.86402226510}, {172.1750442220,32.77311044340}, 7.0710000000, 73.53064455046415, 5e-12 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {147.5492071803, 145.6766243826}, {167.2268895566, 172.1750442220}, 7.0000000000, 33.0174141206, 1e-10 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {118.3536153757, 117.9183400940}, {82.3345858193, 117.5374742226}, 7.0710000000, 278.345433819660797780670691281557, 6e-14 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {118.1359777349, 117.7007024532}, {162.3164188221, 177.5510536798}, 7.0710000000, 76.287384216329215291807486210018, 2e-14 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {128.730553861499998902218067087233, 71.512253475799994362205325160176}, {162.316418822099990393326152116060, 177.551053679799991869003861211240}, 7.070999999999999729993760411162, 211.522748240903837313453550450504, 3e-14 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {14.141414141414143657016211363953, 2.020202020202020332106940259109}, {157.008141055500004767964128404856, 157.292337911700002450743340887129}, 7.071, 323.928772158853632845421088859439, 6e-14 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {118.135977734899995539308292791247, 117.700702453199994579335907474160}, {54.142632100480071244419377762824, 141.587294374629351523253717459738}, 7.071, 242.004121681785619557558675296605, 3e-14 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {11.262519956965983070062975457404, 157.529168625219398336412268690765}, {82.334585819300002640375168994069, 117.537474222599996664939681068063}, 7.071, 87.184299895367630028886196669191, 0 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {11.262519956965983070062975457404, 157.529168625219398336412268690765}, {82.606569658775924835936166346073, 117.727124442220485889265546575189}, 7.071, 87.242967252403175848485261667520, 1e-100 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {153.397994421199996395444031804800, 146.932467108600008032226469367743}, {157.170310847299987244696239940822, 175.262502943700013702255091629922}, 6.0, 28.644049050199150485696009127423, 4e-15 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {152.411714924699992934620240703225, 141.083836012799991976862656883895}, {157.170310847299987244696239940822, 175.262502943700013702255091629922}, 6.0, 34.575257438038448754014098085463, 1e-100 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {153.397994421199996395444031804800, 146.932467108600008032226469367743}, {156.861455436299991106352536007762, 171.174856726000001572174369357526}, 6.0, 24.544751151048469495208337320946, 1e-100 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {152.411714924699992934620240703225, 141.083836012799991976862656883895}, {156.861455436299991106352536007762, 171.174856726000001572174369357526}, 6.0, 30.475959538887764210812747478485, 1e-100 },
    NonStraightPathParams{"poly/exactFitCorridors.poly", {100.125000000000000000000000000000, 27.750000000000000000000000000000}, {29.998066836199999585232944809832, 139.847100408100004642619751393795}, 10.0, 173.513666526867723405302967876196, 1e-100 }
));

INSTANTIATE_TEST_SUITE_P(ExactFitCorridorsSuite,
                         ParamNoPaths,
                         testing::Values(
    NoPathParams{"poly/exactFitCorridors.poly", {68.7320000000, 122.2710000000}, {119.0070000000, 78.3080000000}, 7.5000000000 },
    NoPathParams{"poly/exactFitCorridors.poly", {97.4970207957, 69.5782662458}, {119.0070000000, 78.3080000000}, 7.5000000000 },
    NoPathParams{"poly/exactFitCorridors.poly", {178.4427067982, 81.7887523341}, {119.0070000000, 78.3080000000}, 7.5000000000 }
));

INSTANTIATE_TEST_SUITE_P(FunnelSuite,
                         ParamNonStraightPaths,
                         testing::Values(
    NonStraightPathParams{"poly/funnel.poly", {566.2413676285, 42.4349177499}, {242.6718163068, 347.2562593960}, 15.0000000000, 445.3639142623, 1e-10 },

    // C++ exception with description "createCircleConsciousLine: point2 is inside point1's circle" thrown in the test body.
    NonStraightPathParams{"poly/funnel.poly", {507.0000000000, 1456.0000000000}, {253.0000000000, 1111.0000000000}, 15.0000000000, 434.2396854904, 1e-10 },

    // C++ exception with description "createCircleConsciousLine: point2 is inside point1's circle" thrown in the test body.
    NonStraightPathParams{"poly/funnel.poly", {1037.9673127968, 1462.8488600656}, {253.0000000000, 1111.0000000000}, 15.0000000000, 896.4334393972, 1e-11 },

    // C++ exception with description "createCircleConsciousLine: point2 is inside point1's circle" thrown in the test body.
    NonStraightPathParams{"poly/funnel.poly", {471.0000000000, 1401.0000000000}, {242.6718163068, 347.2562593960}, 15.0000000000, 1128.6913561925, 1e-10 }
));

INSTANTIATE_TEST_SUITE_P(MazeSuite,
                         ParamNonStraightPaths,
                         testing::Values(
    NonStraightPathParams{"poly/maze.poly", {1410.068209694864890479948371648788,1838.250124849350868316832929849625}, {1392.3794497779, 899.9405811957}, 10.0000000000, 938.47626072214314, 1e-10 },
    NonStraightPathParams{"poly/maze.poly", {1411.151870263403452554484829306602, 2181.335755317390066920779645442963}, {3687.452439999999114661477506160736, 2811.805259999999179854057729244232}, 5.1, 3540.237163135314403916709125041962, 1e-12 },
    NonStraightPathParams{"poly/maze.poly", {2086.452440000000024156179279088974, 2519.805259999999634601408615708351}, {1665.452440000000024156179279088974, 2637.805259999999634601408615708351}, 5.1, 466.601393895046953730343375355005, 1e-13 },
    NonStraightPathParams{"poly/maze.poly", {3687.452439999999114661477506160736, 2811.805259999999179854057729244232}, {2086.452440000000024156179279088974, 2519.805259999999634601408615708351}, 5.1, 2471.037299581738352571846917271614, 1e-100 },
    NonStraightPathParams{"poly/maze.poly", {2380.702440000000024156179279088974, 1215.305260000000089348759502172470}, {2074.451904707600078836549073457718, 1830.590553033099922686233185231686}, 5.1, 2423.204573053586045716656371951103, 1e-100 }
));

INSTANTIATE_TEST_SUITE_P(MazeSuite,
                         ParamNoPaths,
                         testing::Values(
    NoPathParams{"poly/maze.poly", {2349.7343410450, 2631.7827683575}, {1159.5382055279, 2151.4818888362}, 50.0000000000 }
));

INSTANTIATE_TEST_SUITE_P(SingleXSuite,
                         ParamNonStraightPaths,
                         testing::Values(
    NonStraightPathParams{"poly/singleX.poly", {776.9777092197, 815.2819340047}, {346.9257309514, 621.5844336713}, 27.0000000000, 472.6627536145, 5e-11 }
));

INSTANTIATE_TEST_SUITE_P(SingleSquareSuite,
                         ParamNonStraightPaths,
                         testing::Values(
    NonStraightPathParams{"poly/singleSquare.poly", {567.8669409821, 580.3716026558}, {468.3216176552, 488.1142363168}, 1.0000000000, 135.7468461402, 1e-10 }
));


INSTANTIATE_TEST_SUITE_P(TwoSquaresSuite,
                         ParamNonStraightPaths,
                         testing::Values(
    NonStraightPathParams{"poly/twoSquares.poly", {4721.152028999999856750946491956711, 4333.546580000000176369212567806244}, {4647.5, 4242.5}, 10.0, 119.379585492524455503371427766979, 1e-100 }
));

INSTANTIATE_TEST_SUITE_P(ThreeSquaresSuite,
                         ParamNonStraightPaths,
                         testing::Values(
    NonStraightPathParams{"poly/threeSquares.poly", {4326.749614000000292435288429260254, 7042.867371999999704712536185979843}, {4492.154220999999779451172798871994, 7055.925629999999728170223534107208}, 10.0, 166.000796341705381564679555594921, 1e-100 }
));

INSTANTIATE_TEST_SUITE_P(Misc1Suite,
                         ParamNonStraightPaths,
                         testing::Values(
    NonStraightPathParams{"poly/misc1.poly", {1353.156337177544173755450174212456, 1296.291576411703090343507938086987}, {1472.312372080794148132554255425930, 1371.190519313953473101719282567501}, 10.0, 142.363430959989955226774327456951, 3e-14 }
));

INSTANTIATE_TEST_SUITE_P(Misc2Suite,
                         ParamNonStraightPaths,
                         testing::Values(
    NonStraightPathParams{"poly/misc2.poly", {1353.156337177544173755450174212456, 1296.291576411703090343507938086987}, {1472.312372080794148132554255425930, 1371.190519313953473101719282567501}, 10.0, 142.363430959989955226774327456951, 3e-14 },
    NonStraightPathParams{"poly/misc2.poly", {1337.700000000000045474735088646412, 1342.700000000000045474735088646412}, {1467.700000000000045474735088646412, 1377.700000000000045474735088646412}, 10.0, 152.396540840325030785606941208243, 1e-100 },
    NonStraightPathParams{"poly/misc2.poly", {1328.687697769322312524309381842613, 1240.157645440762053112848661839962}, {1467.700000000000045474735088646412, 1377.700000000000045474735088646412}, 10.0, 196.321360627075932825391646474600, 3e-14 }
));

INSTANTIATE_TEST_SUITE_P(Misc3Suite,
                         ParamNonStraightPaths,
                         testing::Values(
    NonStraightPathParams{"poly/misc3.poly", {156.718102029393804741630447097123, 1692.014938642568722571013495326042}, {286.359754262830676907469751313329, 1631.317574747015669345273636281490}, 10.0, 143.875056961459108606504742056131, 1e-100 },
    NonStraightPathParams{"poly/misc3.poly", {206.373734505493473534443182870746, 1788.222429097055055535747669637203}, {286.359754262830676907469751313329, 1631.317574747015669345273636281490}, 10.0, 176.698522943846796806610655039549, 1e-100 },
    NonStraightPathParams{"poly/misc3.poly", {262.938488846611789995222352445126, 1628.678558925469815221731550991535}, {206.373734505493473534443182870746, 1788.222429097055055535747669637203}, 10.0, 169.441579533560911841050256043673, 1e-100 },
    NonStraightPathParams{"poly/misc3.poly", {230.528075788252465372352162376046, 1634.121529057407997242989949882030}, {134.377197596249573052773484960198, 1710.163025921769531123572960495949}, 10.0, 122.585890796886090470252383965999, 1e-100 },
    NonStraightPathParams{"poly/misc3.poly", {230.528075788252465372352162376046, 1634.121529057407997242989949882030}, {282.159562831013374761823797598481, 1687.427277424113526649307459592819}, 10.0, 74.211274501798698111088015139103, 1e-100 },
    NonStraightPathParams{"poly/misc3.poly", {230.528075788252465372352162376046, 1634.121529057407997242989949882030}, {198.416222531313877652792143635452, 1781.022775406130676856264472007751}, 10.0, 165.126694496206141593575011938810, 3e-14 },
    NonStraightPathParams{"poly/misc3.poly", {230.528075788252465372352162376046, 1634.121529057407997242989949882030}, {155.976158669022737512932508252561, 1652.186867252746651502093300223351}, 10.0, 76.709483053038880484564288053662, 1e-100 },
    NonStraightPathParams{"poly/misc3.poly", {230.528075788252465372352162376046, 1634.121529057407997242989949882030}, {174.531458824827865328188636340201, 1631.977328702402019189321435987949}, 10.0, 56.037654363034320681435929145664, 1e-100 },
    NonStraightPathParams{"poly/misc3.poly", {230.528075788252465372352162376046, 1634.121529057407997242989949882030}, {262.5, 1699.25}, 10.0, 72.688340966484545901948877144605, 1e-100 }
));

INSTANTIATE_TEST_SUITE_P(Misc4Suite,
                         ParamNonStraightPaths,
                         testing::Values(
  NonStraightPathParams{"poly/misc4.poly", {502.0, 614.0}, {415.0, 819.0}, 100.0, 931.586758488275904710462782531977, 1e-100 }
));

// ===================================================================================================================
// ===================================================================================================================
// ===================================================================================================================

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // Initialize abseil logging.
  absl::InitializeLog();
  // Ideally, this setting of the threshold and vlog level should come from the commandline, but something about our cmake setup results in the relevant flags not being linked into this binary.
  absl::SetStderrThreshold(absl::LogSeverityAtLeast::kInfo);
  absl::SetGlobalVLogLevel(10);
  return RUN_ALL_TESTS();
}
