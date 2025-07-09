#include "dstereo_occnet/thread_pool.h"

ThreadPool::ThreadPool(size_t num_threads) : stop_flag_(false) {
  for (size_t i = 0; i < num_threads; ++i) {
    workers_.emplace_back([this]() {
      while (true) {
        std::function<void()> task;
        {
          std::unique_lock<std::mutex> lock(queue_mutex_);
          condition_.wait(lock, [this]() { return stop_flag_ || !tasks_.empty(); });

          if (stop_flag_ && tasks_.empty())
            return;

          task = std::move(tasks_.front());
          tasks_.pop();
        }
        task();
      }
    });
  }
}

ThreadPool::~ThreadPool() {
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    stop_flag_ = true;
  }
  condition_.notify_all();
  for (std::thread &worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
}

void ThreadPool::enqueue(std::function<void()> task) {
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    tasks_.push(std::move(task));
  }
  condition_.notify_one();
}