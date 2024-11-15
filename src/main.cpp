#include "RazorAHRS.h"
#include "UST10LX.h"
#include "pathing.h"
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <ostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <utility>

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

const float MULTIPLIER = 0.01;

float speed;
float steeringAngle;

struct Atomics {
  double accx, accy, angularVelocity;
};

Atomics sharedValues{0.0, 0.0, 0.0};
std::mutex valueMutex;

std::pair<double, double> car_position{0.0f, 0.0f};
float car_angle{0.0f};
std::pair<float, float> car_velocity{0.0f, 0.0f};

RazorAHRS *razor;

const std::string serial_port_name = "/dev/ttyUSB0"; // a good guess on linux

// Razor error callback handler
// Will be called from (and in) Razor background thread!
void on_error(const std::string &msg) {
  std::cout << "  " << "ERROR: " << msg << std::endl;

  // NOTE: make a copy of the message if you want to save it or send it to
  // another thread. Do not save or pass the reference itself, it will not be
  // valid after this function returns!
}

// Razor data callback handler
// Will be called from (and in) Razor background thread!
// 'data' depends on mode that was set when creating the RazorAHRS object. In
// this case 'data' holds 3 float values: yaw, pitch and roll.
void on_data(const float data[]) {
  // If you created the Razor object using RazorAHRS::ACC_MAG_GYR_RAW or
  // RazorAHRS::ACC_MAG_GYR_CALIBRATED instead of RazorAHRS::YAW_PITCH_ROLL,
  // 'data' would contain 9 values that could be printed like this:

  std::unique_lock<std::mutex> lock(valueMutex); // Lock the mutex
  sharedValues.accx = data[0];
  sharedValues.accy = data[2];
  sharedValues.angularVelocity = data[7];
  lock.unlock(); // unlock
}

void update(std::vector<DataPoint> lidar_samples, double deltaTime) {
  Atomics localCopy;
  std::unique_lock<std::mutex> lock(valueMutex); // Lock the mutex
  localCopy = sharedValues;
  lock.unlock(); // unlock
  // angle = angle + car.angularVelocity * deltaTime;
  // std::array<std::array<float, 2>, 2> rotation_matrix = {
  //     {{std::cos(angle), -std::sin(angle)},
  //      {std::sin(angle), std::cos(angle)}}};
  // auto acc_local =
  //     std::array<float, 2>{rc::physics::get_linear_acceleration()[0],
  //                          rc::physics::get_linear_acceleration()[2]};
  // std::array<float, 2> acc_global{rotation_matrix[0][0] * acc_local[0] +
  //                                     rotation_matrix[0][1] * acc_local[1],
  //                                 rotation_matrix[1][0] * acc_local[0] +
  //                                     rotation_matrix[1][1] * acc_local[1]};
  // velocity[0] += acc_global[0] * deltaTime;
  // velocity[1] += acc_global[1] * deltaTime;
  // position[0] += velocity[0] * deltaTime;
  // position[1] += velocity[1] * deltaTime;

  for (auto &row : occupancy_grid) {
    std::fill(row.begin(), row.end(), 0);
  }

  map_size max_y = 0;
  std::vector<std::pair<int, int>> directions = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  // #pragma omp parallel for // Parallelize loop if OpenMP is available
  for (uint32_t i = 0; i < lidar_samples.size(); ++i) {
    // if (lidar_samples[i].distance < 5)
    //   continue;

    float x_local = lidar_samples[i].distance * cos(lidar_samples[i].angle);
    float y_local = lidar_samples[i].distance * sin(lidar_samples[i].angle);
    map_size y_coord = static_cast<map_size>(x_local * MULTIPLIER);
    map_size x_coord =
        static_cast<map_size>(y_local * MULTIPLIER) + MAP_WIDTH / 2;

    if (!(x_coord >= 0 && x_coord < MAP_WIDTH && y_coord >= 0 &&
          y_coord < MAP_HEIGHT)) {
      continue;
    }
    std::cout << x_coord << " " << y_coord << std::endl;
    max_y = std::max(max_y, y_coord);

    occupancy_grid[x_coord][y_coord] = 9;

#define INNER_SIZE 2
#define OUTER_SIZE 2
    for (const auto &[dx, dy] : directions) {
#if (OUTER_SIZE > 0)
      for (int r = INNER_SIZE; r <= OUTER_SIZE + INNER_SIZE; ++r) {
        if (x_coord + dx * r >= 0 && x_coord + dx * r < MAP_WIDTH &&
            y_coord + dy * r >= 0 && y_coord + dy * r < MAP_HEIGHT)
          occupancy_grid[x_coord + dx * r][y_coord + dy * r] = 5;
      }
#endif

#if (INNER_SIZE > 1)
      for (int r = 1; r <= INNER_SIZE; ++r) {
        if (x_coord + dx * r >= 0 && x_coord + dx * r < MAP_WIDTH &&
            y_coord + dy * r >= 0 && y_coord + dy * r < MAP_HEIGHT)
          occupancy_grid[x_coord + dx * r][y_coord + dy * r] = 9;
      }
#endif
    }
  }

  // std::cout << "new" << std::endl;
  // int buffer_size = 2;
  // int buffer_size_2 = 0;
  // this could be more efficient
  // add_concentric_plus_buffers(occupancy_grid, buffer_size, buffer_size_2);
  // other A*
  // Node start;
  // start.x = MAP_WIDTH / 2;
  // start.y = 0;
  // Node end;
  // end.x = MAP_WIDTH / 2;
  // end.y = max_y;
  // std::vector<glm::vec2> path = {};
  // for (Node node : Cordinate::aStar2(start, end, occupancy_grid)) {
  //   // Your code here
  //   path.push_back({node.x, node.y});
  //   // occupancy_grid[node.x][node.y] = 2;
  //   // std::cout << node.x << " " << node.y << std::endl;
  // }

  max_y = std::min(MAP_HEIGHT - 1, max_y + 5 + INNER_SIZE + OUTER_SIZE);
  std::pair<map_size, map_size> start = {MAP_WIDTH / 2, 0};
  std::pair<map_size, map_size> end = {MAP_WIDTH / 2, max_y};
  std::vector<std::pair<map_size, map_size>> path = astar(start, end);

#define DEBUG_MODE_SIM
#ifdef DEBUG_MODE_SIM
  occupancy_grid[start.first][start.second] = 1;
  occupancy_grid[end.first][end.second] = 1;

  for (uint32_t i = 0; i < path.size(); i++) {
    occupancy_grid[path[i].first][path[i].second] = 8;
  }
  for (map_size i = 0; i < occupancy_grid.size(); i++) {
    for (map_size j = 0; j < max_y + 1; j++) {
      std::cout << (int) occupancy_grid[i][j];
    }
    std::cout << std::endl;
  }
#endif

  const map_size car_size = 4;
  float heading = M_PI / 2.0f;
  if (path.size() > car_size) {
    heading =
        atan2(path[car_size].first, path[car_size].second - MAP_WIDTH / 2.0f);
  } else if (path.size() > 0) {
    heading = atan2(path[path.size() - 1].second,
                    path[path.size() - 1].first - MAP_WIDTH / 2.0f);
  }

  float angle = heading - M_PI / 2.0f;

  speed = 0.1f;
  float multiplier = -4.0f;
  float clamped_angle = std::max(std::min(angle * multiplier, 1.0f), -1.0f);
  steeringAngle = clamped_angle;
}

void mainLoop() {
  uint32_t f = 0;

  std::chrono::duration<double> secondsPerFrame =
      std::chrono::duration<double>::zero();
  std::chrono::duration<double> secondsPerComputation =
      std::chrono::duration<double>::zero();
  std::chrono::duration<double> frameTimes =
      std::chrono::duration<double>::zero();
  std::chrono::duration<double> frameTimeCalc =
      std::chrono::duration<double>::zero();

  std::chrono::time_point currentTime = std::chrono::system_clock::now();
  std::chrono::time_point lastTime = currentTime;

  std::chrono::time_point currentCalc = currentTime;

  UST10LX LiDAR = UST10LX(-135);

  std::cout << "Trying to connect to UST10LX" << std::endl;
  LiDAR.connect("192.168.1.11");

  while (!LiDAR) {
    std::cout << "Retrying connection in 5 seconds" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    LiDAR.connect("192.168.1.11");
  }

  std::cout << "Connection successful!" << std::endl;

  while (true) {
    // the first frame time is not accurate but whatever
    std::chrono::time_point currentTime = std::chrono::system_clock::now();
    secondsPerFrame = currentTime - lastTime;
    lastTime = currentTime;
    frameTimes += secondsPerFrame;
    f++;
    // Print the number of seconds for 1000 frames
    if (f == 1000) {
      printf("Frame time: %f, Computation time:%f\n", frameTimes.count(),
             frameTimeCalc.count());
      f = 0;
      frameTimes = std::chrono::duration<double>::zero();
      frameTimeCalc = std::chrono::duration<double>::zero();
    }

    speed = 0.0f;
    steeringAngle = 0.0f;
    // autonomous driving
    currentCalc = std::chrono::system_clock::now();
    if (LiDAR.scan()) {
      update(LiDAR.getDataPoints(), secondsPerFrame.count());
    }
    secondsPerComputation = std::chrono::system_clock::now() - currentCalc;
    frameTimeCalc += secondsPerComputation;
#ifdef DEBUG_MODE_SIM
    break;
#endif
  }
}

int main() {
  try {
    // razor = new RazorAHRS(serial_port_name, on_data, on_error,
    //                       RazorAHRS::ACC_MAG_GYR_RAW);
    mainLoop();
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}