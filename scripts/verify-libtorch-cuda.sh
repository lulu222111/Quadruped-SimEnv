#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

mkdir -p artifacts/logs

LOG="$REPO_ROOT/artifacts/logs/libtorch-cuda-verification-$(date +%Y%m%d-%H%M%S).log"

echo "验证日志：$LOG"

set +e
sudo docker compose \
    --file compose.yaml \
    run \
    --rm \
    --entrypoint bash \
    simenv \
    -s 2>&1 <<'CONTAINER_SCRIPT' | tee "$LOG"

set -Eeo pipefail

TEST_DIR=/tmp/libtorch-cuda-smoke
rm -rf "$TEST_DIR"
mkdir -p "$TEST_DIR"
cd "$TEST_DIR"

cat > CMakeLists.txt <<'CMAKE'
cmake_minimum_required(VERSION 3.14)
project(torch_cuda_smoke LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Torch REQUIRED CONFIG)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_executable(torch_cuda_smoke main.cpp)
target_link_libraries(torch_cuda_smoke PRIVATE "${TORCH_LIBRARIES}")
CMAKE

cat > main.cpp <<'CPP'
#include <torch/torch.h>
#include <torch/cuda.h>

#include <iostream>

int main() {
    std::cout << "LibTorch version: " << TORCH_VERSION << '\n';
    std::cout << "CUDA available: "
              << std::boolalpha
              << torch::cuda::is_available()
              << '\n';
    std::cout << "CUDA device count: "
              << static_cast<int>(torch::cuda::device_count())
              << '\n';

    if (!torch::cuda::is_available()) {
        std::cerr << "CUDA is not available to LibTorch\n";
        return 2;
    }

    auto options = torch::TensorOptions()
        .dtype(torch::kFloat32)
        .device(torch::kCUDA);

    auto a = torch::rand({1024, 1024}, options);
    auto b = torch::rand({1024, 1024}, options);
    auto c = torch::matmul(a, b);

    const double checksum = c.sum().item<double>();
    const bool finite = torch::isfinite(c).all().item<bool>();

    std::cout << "Tensor device: " << c.device() << '\n';
    std::cout << "Result shape: " << c.sizes() << '\n';
    std::cout << "Finite result: " << std::boolalpha << finite << '\n';
    std::cout << "Checksum: " << checksum << '\n';

    return finite ? 0 : 3;
}
CPP

cmake \
    -S . \
    -B build \
    -DCMAKE_PREFIX_PATH=/opt/libtorch \
    -DCMAKE_BUILD_TYPE=Release

cmake --build build --parallel 2
./build/torch_cuda_smoke
CONTAINER_SCRIPT

VERIFY_STATUS="${PIPESTATUS[0]}"
set -e

echo
echo "LibTorch CUDA验证退出码：$VERIFY_STATUS"
echo "完整日志：$LOG"

exit "$VERIFY_STATUS"