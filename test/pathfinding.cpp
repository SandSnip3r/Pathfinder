#include "behaviorBuilder.h"
#include "pathfinder.h"
#include "triangle_lib_navmesh.h"
#include "vector.h"

#include "triangle/triangle_api.h"

#include <gtest/gtest.h>

class TestFixtureFromFile : public ::testing::Test {
protected:
  void initFromFile(const std::string &filename) {
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

private:
  void initTriangleData(const std::string &filename, triangle::triangleio &triangleData, triangle::triangleio &triangleVoronoiData) {
    
    triangle::triangleio inputData;
    triangle::triangle_initialize_triangleio(&inputData);

    int firstNode;
    int readFileResult = triangle::triangle_read_poly(filename.c_str(), &inputData, &firstNode);
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

class StartTouchingVertexTest : public TestFixtureFromFile {
protected:
  StartTouchingVertexTest() {
    // Call function from base class
    initFromFile(kPolyFilename_);
  }
  void SetUp() override {}
  void TearDown() override {}
  static constexpr const double kAgentRadius_{10.0};
private:
  const std::string kPolyFilename_{"test/startTouchingVertexTest.poly"};
};

class ExactFitIonTrapTest : public TestFixtureFromFile {
protected:
  ExactFitIonTrapTest() {
    // Call function from base class
    initFromFile(kPolyFilename_);
  }
  void SetUp() override {}
  void TearDown() override {}
  static constexpr const double kAgentRadius_{10.0};
private:
  const std::string kPolyFilename_{"test/exactFitIonTrap.poly"};
};

class ExactFitCorridors : public TestFixtureFromFile {
protected:
  ExactFitCorridors() {
    // Call function from base class
    initFromFile(kPolyFilename_);
  }
  void SetUp() override {}
  void TearDown() override {}
  static constexpr const double kAgentRadius_{10.0};
private:
  const std::string kPolyFilename_{"test/exactFitCorridors.poly"};
};

// ===================================================================================================================
// ============================================= StartTouchingVertexTest =============================================
// ===================================================================================================================

TEST_F(StartTouchingVertexTest, StartTopGoingLeft) {
  const pathfinder::Vector startPoint{500.0, 750.0-kAgentRadius_};
  const pathfinder::Vector goalPoint{400.0, 750.0-kAgentRadius_};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_EQ(pathLength, 100.0);
}

TEST_F(StartTouchingVertexTest, StartTopGoingRight) {
  const pathfinder::Vector startPoint{500.0, 750.0-kAgentRadius_};
  const pathfinder::Vector goalPoint{600.0, 750.0-kAgentRadius_};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_EQ(pathLength, 100.0);
}

TEST_F(StartTouchingVertexTest, StartBottomGoingLeft) {
  const pathfinder::Vector startPoint{500.0, 250.0+kAgentRadius_};
  const pathfinder::Vector goalPoint{400.0, 250.0+kAgentRadius_};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_EQ(pathLength, 100.0);
}

TEST_F(StartTouchingVertexTest, StartBottomGoingRight) {
  const pathfinder::Vector startPoint{500.0, 250.0+kAgentRadius_};
  const pathfinder::Vector goalPoint{600.0, 250.0+kAgentRadius_};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_EQ(pathLength, 100.0);
}

// ===================================================================================================================
// =============================================== ExactFitIonTrapTest ===============================================
// ===================================================================================================================

TEST_F(ExactFitIonTrapTest, StartBottomLeftGoingBottomRight) {
  const pathfinder::Vector startPoint{15.0, 30.0};
  const pathfinder::Vector goalPoint{85.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

TEST_F(ExactFitIonTrapTest, StartBottomLeftGoingTopRight) {
  const pathfinder::Vector startPoint{15.0, 30.0};
  const pathfinder::Vector goalPoint{85.0, 70.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

TEST_F(ExactFitIonTrapTest, StartTopLeftGoingBottomRight) {
  const pathfinder::Vector startPoint{15.0, 70.0};
  const pathfinder::Vector goalPoint{85.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

TEST_F(ExactFitIonTrapTest, StartTopLeftGoingTopRight) {
  const pathfinder::Vector startPoint{15.0, 70.0};
  const pathfinder::Vector goalPoint{85.0, 70.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

TEST_F(ExactFitIonTrapTest, StartBottomRightGoingBottomLeft) {
  const pathfinder::Vector startPoint{85.0, 30.0};
  const pathfinder::Vector goalPoint{15.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

TEST_F(ExactFitIonTrapTest, StartBottomRightGoingTopLeft) {
  const pathfinder::Vector startPoint{85.0, 30.0};
  const pathfinder::Vector goalPoint{15.0, 70.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

TEST_F(ExactFitIonTrapTest, StartTopRightGoingBottomLeft) {
  const pathfinder::Vector startPoint{85.0, 70.0};
  const pathfinder::Vector goalPoint{15.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

TEST_F(ExactFitIonTrapTest, StartTopRightGoingTopLeft) {
  const pathfinder::Vector startPoint{85.0, 70.0};
  const pathfinder::Vector goalPoint{15.0, 70.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_GT(pathLength, 70.0);
  EXPECT_LT(pathLength, 90.0);
}

// ===================================================================================================================
// ================================================ ExactFitCorridors ================================================
// ===================================================================================================================

TEST_F(ExactFitCorridors, StartLeftTopGoingLeftBottom) {
  const pathfinder::Vector startPoint{30.0, 140.0};
  const pathfinder::Vector goalPoint{30.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}

TEST_F(ExactFitCorridors, StartLeftTopGoingRightTop) {
  const pathfinder::Vector startPoint{30.0, 140.0};
  const pathfinder::Vector goalPoint{170.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartLeftTopGoingRightBottom) {
  const pathfinder::Vector startPoint{30.0, 140.0};
  const pathfinder::Vector goalPoint{170.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartLeftTopGoingBottomLeft) {
  const pathfinder::Vector startPoint{30.0, 140.0};
  const pathfinder::Vector goalPoint{60.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartLeftTopGoingBottomRight) {
  const pathfinder::Vector startPoint{30.0, 140.0};
  const pathfinder::Vector goalPoint{140.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartLeftTopGoingTopLeft) {
  const pathfinder::Vector startPoint{30.0, 140.0};
  const pathfinder::Vector goalPoint{60.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartLeftTopGoingTopRight) {
  const pathfinder::Vector startPoint{30.0, 140.0};
  const pathfinder::Vector goalPoint{140.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartLeftBottomGoingLeftTop) {
  const pathfinder::Vector startPoint{30.0, 60.0};
  const pathfinder::Vector goalPoint{30.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}

TEST_F(ExactFitCorridors, StartLeftBottomGoingRightTop) {
  const pathfinder::Vector startPoint{30.0, 60.0};
  const pathfinder::Vector goalPoint{170.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartLeftBottomGoingRightBottom) {
  const pathfinder::Vector startPoint{30.0, 60.0};
  const pathfinder::Vector goalPoint{170.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartLeftBottomGoingBottomLeft) {
  const pathfinder::Vector startPoint{30.0, 60.0};
  const pathfinder::Vector goalPoint{60.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartLeftBottomGoingBottomRight) {
  const pathfinder::Vector startPoint{30.0, 60.0};
  const pathfinder::Vector goalPoint{140.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartLeftBottomGoingTopLeft) {
  const pathfinder::Vector startPoint{30.0, 60.0};
  const pathfinder::Vector goalPoint{60.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartLeftBottomGoingTopRight) {
  const pathfinder::Vector startPoint{30.0, 60.0};
  const pathfinder::Vector goalPoint{140.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartRightTopGoingLeftTop) {
  const pathfinder::Vector startPoint{170.0, 140.0};
  const pathfinder::Vector goalPoint{30.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartRightTopGoingLeftBottom) {
  const pathfinder::Vector startPoint{170.0, 140.0};
  const pathfinder::Vector goalPoint{30.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartRightTopGoingRightBottom) {
  const pathfinder::Vector startPoint{170.0, 140.0};
  const pathfinder::Vector goalPoint{170.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}

TEST_F(ExactFitCorridors, StartRightTopGoingBottomLeft) {
  const pathfinder::Vector startPoint{170.0, 140.0};
  const pathfinder::Vector goalPoint{60.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartRightTopGoingBottomRight) {
  const pathfinder::Vector startPoint{170.0, 140.0};
  const pathfinder::Vector goalPoint{140.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartRightTopGoingTopLeft) {
  const pathfinder::Vector startPoint{170.0, 140.0};
  const pathfinder::Vector goalPoint{60.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartRightTopGoingTopRight) {
  const pathfinder::Vector startPoint{170.0, 140.0};
  const pathfinder::Vector goalPoint{140.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartRightBottomGoingLeftTop) {
  const pathfinder::Vector startPoint{170.0, 60.0};
  const pathfinder::Vector goalPoint{30.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartRightBottomGoingLeftBottom) {
  const pathfinder::Vector startPoint{170.0, 60.0};
  const pathfinder::Vector goalPoint{30.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartRightBottomGoingRightTop) {
  const pathfinder::Vector startPoint{170.0, 60.0};
  const pathfinder::Vector goalPoint{170.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}

TEST_F(ExactFitCorridors, StartRightBottomGoingBottomLeft) {
  const pathfinder::Vector startPoint{170.0, 60.0};
  const pathfinder::Vector goalPoint{60.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartRightBottomGoingBottomRight) {
  const pathfinder::Vector startPoint{170.0, 60.0};
  const pathfinder::Vector goalPoint{140.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartRightBottomGoingTopLeft) {
  const pathfinder::Vector startPoint{170.0, 60.0};
  const pathfinder::Vector goalPoint{60.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartRightBottomGoingTopRight) {
  const pathfinder::Vector startPoint{170.0, 60.0};
  const pathfinder::Vector goalPoint{140.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartBottomLeftGoingLeftTop) {
  const pathfinder::Vector startPoint{60.0, 30.0};
  const pathfinder::Vector goalPoint{30.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartBottomLeftGoingLeftBottom) {
  const pathfinder::Vector startPoint{60.0, 30.0};
  const pathfinder::Vector goalPoint{30.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartBottomLeftGoingRightTop) {
  const pathfinder::Vector startPoint{60.0, 30.0};
  const pathfinder::Vector goalPoint{170.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartBottomLeftGoingRightBottom) {
  const pathfinder::Vector startPoint{60.0, 30.0};
  const pathfinder::Vector goalPoint{170.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartBottomLeftGoingBottomRight) {
  const pathfinder::Vector startPoint{60.0, 30.0};
  const pathfinder::Vector goalPoint{140.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}

TEST_F(ExactFitCorridors, StartBottomLeftGoingTopLeft) {
  const pathfinder::Vector startPoint{60.0, 30.0};
  const pathfinder::Vector goalPoint{60.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartBottomLeftGoingTopRight) {
  const pathfinder::Vector startPoint{60.0, 30.0};
  const pathfinder::Vector goalPoint{140.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartBottomRightGoingLeftTop) {
  const pathfinder::Vector startPoint{140.0, 30.0};
  const pathfinder::Vector goalPoint{30.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartBottomRightGoingLeftBottom) {
  const pathfinder::Vector startPoint{140.0, 30.0};
  const pathfinder::Vector goalPoint{30.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartBottomRightGoingRightTop) {
  const pathfinder::Vector startPoint{140.0, 30.0};
  const pathfinder::Vector goalPoint{170.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartBottomRightGoingRightBottom) {
  const pathfinder::Vector startPoint{140.0, 30.0};
  const pathfinder::Vector goalPoint{170.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartBottomRightGoingBottomLeft) {
  const pathfinder::Vector startPoint{140.0, 30.0};
  const pathfinder::Vector goalPoint{60.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}

TEST_F(ExactFitCorridors, StartBottomRightGoingTopLeft) {
  const pathfinder::Vector startPoint{140.0, 30.0};
  const pathfinder::Vector goalPoint{60.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartBottomRightGoingTopRight) {
  const pathfinder::Vector startPoint{140.0, 30.0};
  const pathfinder::Vector goalPoint{140.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartTopLeftGoingLeftTop) {
  const pathfinder::Vector startPoint{60.0, 170.0};
  const pathfinder::Vector goalPoint{30.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartTopLeftGoingLeftBottom) {
  const pathfinder::Vector startPoint{60.0, 170.0};
  const pathfinder::Vector goalPoint{30.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartTopLeftGoingRightTop) {
  const pathfinder::Vector startPoint{60.0, 170.0};
  const pathfinder::Vector goalPoint{170.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartTopLeftGoingRightBottom) {
  const pathfinder::Vector startPoint{60.0, 170.0};
  const pathfinder::Vector goalPoint{170.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartTopLeftGoingBottomLeft) {
  const pathfinder::Vector startPoint{60.0, 170.0};
  const pathfinder::Vector goalPoint{60.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartTopLeftGoingBottomRight) {
  const pathfinder::Vector startPoint{60.0, 170.0};
  const pathfinder::Vector goalPoint{140.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartTopLeftGoingTopRight) {
  const pathfinder::Vector startPoint{60.0, 170.0};
  const pathfinder::Vector goalPoint{140.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}

TEST_F(ExactFitCorridors, StartTopRightGoingLeftTop) {
  const pathfinder::Vector startPoint{140.0, 170.0};
  const pathfinder::Vector goalPoint{30.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartTopRightGoingLeftBottom) {
  const pathfinder::Vector startPoint{140.0, 170.0};
  const pathfinder::Vector goalPoint{30.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartTopRightGoingRightTop) {
  const pathfinder::Vector startPoint{140.0, 170.0};
  const pathfinder::Vector goalPoint{170.0, 140.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartTopRightGoingRightBottom) {
  const pathfinder::Vector startPoint{140.0, 170.0};
  const pathfinder::Vector goalPoint{170.0, 60.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 207.1238898038, 1e-6);
}

TEST_F(ExactFitCorridors, StartTopRightGoingBottomLeft) {
  const pathfinder::Vector startPoint{140.0, 170.0};
  const pathfinder::Vector goalPoint{60.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartTopRightGoingBottomRight) {
  const pathfinder::Vector startPoint{140.0, 170.0};
  const pathfinder::Vector goalPoint{140.0, 30.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 211.4159265359, 1e-6);
}

TEST_F(ExactFitCorridors, StartTopRightGoingTopLeft) {
  const pathfinder::Vector startPoint{140.0, 170.0};
  const pathfinder::Vector goalPoint{60.0, 170.0};

  ASSERT_TRUE(navmesh_.has_value());
  pathfinder::Pathfinder<pathfinder::navmesh::TriangleLibNavmesh> pathfinder(*navmesh_, kAgentRadius_);
  const auto pathfindingResult = pathfinder.findShortestPath(startPoint, goalPoint);

  const auto pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
  EXPECT_NEAR(pathLength, 80.0000000000, 1e-6);
}
