#include "behaviorBuilder.h"
#include "pathfinder.h"
#include "triangle_lib_navmesh.h"
#include "vector.h"

#include "triangle/triangle_api.h"

#include <gtest/gtest.h>

class StartTouchingVertexTest : public ::testing::Test {
protected:
  StartTouchingVertexTest() {
    // Initialize triangle data
    triangle::triangleio triangleData;
    triangle::triangleio triangleVoronoiData;
    triangle::triangle_initialize_triangleio(&triangleData);
    triangle::triangle_initialize_triangleio(&triangleVoronoiData);
    initTriangleData(triangleData, triangleVoronoiData);

    // Build navmesh from triangle data
    navmesh_ = new pathfinder::navmesh::TriangleLibNavmesh(triangleData, triangleVoronoiData);

    // Free triangle data
    triangle_free_triangleio(&triangleData);
    triangle_free_triangleio(&triangleVoronoiData);
  }

  void SetUp() override {}
  void TearDown() override {}
  
  ~StartTouchingVertexTest() {
    delete navmesh_;
  }
  
  static constexpr const double kAgentRadius_{10.0};
  pathfinder::navmesh::TriangleLibNavmesh *navmesh_{nullptr};
private:
  const std::string polyFilename_{"test/startTouchingVertexTest.poly"};

  void initTriangleData(triangle::triangleio &triangleData, triangle::triangleio &triangleVoronoiData) {
    
    triangle::triangleio inputData;
    triangle::triangle_initialize_triangleio(&inputData);

    int firstNode;
    int readFileResult = triangle::triangle_read_poly(polyFilename_.c_str(), &inputData, &firstNode);
    if (readFileResult < 0) {
      throw std::runtime_error("Unable to open .poly file, error "+std::to_string(readFileResult));
    }

    // Create a context
    triangle::context *ctx;
    ctx = triangle::triangle_context_create();
    // Set context's behavior
    *(ctx->b) = pathfinder::BehaviorBuilder().getBehavior(); // Default behavior

    // Build the triangle mesh
    int meshCreateResult = triangle::triangle_mesh_create(ctx, &inputData);
    if (meshCreateResult < 0) {
      // Free memory
      triangle_free_triangleio(&inputData);
      throw std::runtime_error("Error creating navmesh ("+std::to_string(meshCreateResult)+")");
    }

    // Now, the context holds the mesh, lets extract this data
    // Copy data
    int copyResult = triangle::triangle_mesh_copy(ctx, &triangleData, 1, 1, &triangleVoronoiData);
    if (copyResult < 0) {
      // Free memory
      triangle_free_triangleio(&inputData);
      throw std::runtime_error("Error copying navmesh data ("+std::to_string(copyResult)+")");
    }

    // Done with input data
    triangle_free_triangleio(&inputData);

    // Done with context
    triangle_context_destroy(ctx);
  }
};

TEST_F(StartTouchingVertexTest, StartTopGoingLeft) {
  const pathfinder::Vector startPoint{500.0, 750.0-kAgentRadius_};
  const pathfinder::Vector goalPoint{400.0, 750.0-kAgentRadius_};

  ASSERT_NE(navmesh_, nullptr);
  pathfinder::Pathfinder pathfinder(reinterpret_cast<pathfinder::navmesh::AStarNavmeshInterface&>(*navmesh_), kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);  
  EXPECT_EQ(pathLength, 100.0);
}

TEST_F(StartTouchingVertexTest, StartTopGoingRight) {
  const pathfinder::Vector startPoint{500.0, 750.0-kAgentRadius_};
  const pathfinder::Vector goalPoint{600.0, 750.0-kAgentRadius_};

  ASSERT_NE(navmesh_, nullptr);
  pathfinder::Pathfinder pathfinder(reinterpret_cast<pathfinder::navmesh::AStarNavmeshInterface&>(*navmesh_), kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);  
  EXPECT_EQ(pathLength, 100.0);
}

TEST_F(StartTouchingVertexTest, StartBottomGoingLeft) {
  const pathfinder::Vector startPoint{500.0, 250.0+kAgentRadius_};
  const pathfinder::Vector goalPoint{400.0, 250.0+kAgentRadius_};

  ASSERT_NE(navmesh_, nullptr);
  pathfinder::Pathfinder pathfinder(reinterpret_cast<pathfinder::navmesh::AStarNavmeshInterface&>(*navmesh_), kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);  
  EXPECT_EQ(pathLength, 100.0);
}

TEST_F(StartTouchingVertexTest, StartBottomGoingRight) {
  const pathfinder::Vector startPoint{500.0, 250.0+kAgentRadius_};
  const pathfinder::Vector goalPoint{600.0, 250.0+kAgentRadius_};

  ASSERT_NE(navmesh_, nullptr);
  pathfinder::Pathfinder pathfinder(reinterpret_cast<pathfinder::navmesh::AStarNavmeshInterface&>(*navmesh_), kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);  
  EXPECT_EQ(pathLength, 100.0);
}