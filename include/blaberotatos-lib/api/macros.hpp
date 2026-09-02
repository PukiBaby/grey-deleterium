#include "main.h"

#include <atomic>

class macro_class
{
    public:
        std::atomic<bool> macro_running{false};

        // Creates the task the first time; it is safe to call again because it does nothing
        void start(std::function<void()> macro_routine, std::function<void()> reset_function);
        void run(); // Tells the task to execute one macro

    private:
        void macro_worker(std::function<void()> macro_worker, std::function<void()> reset_function);
        std::optional<pros::Task> macro_task;
};