// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Kai
// Standalone C++ ownership regression: this target must not link libt26x.
#include <t26x/t26x.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

struct t26x_device { int unused; };

namespace {
t26x_device fake_handle{};
bool alive = false;
unsigned closes = 0;
unsigned duplicate_closes = 0;
unsigned resets = 0;
unsigned checks = 0;
t26x_status close_result = T26X_OK;
t26x_status reset_result = T26X_OK;

void require(bool condition, const char *message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

void new_scenario() {
    require(!alive, "previous scenario must release its fake handle");
    closes = 0;
    duplicate_closes = 0;
    resets = 0;
    close_result = T26X_OK;
    reset_result = T26X_OK;
}

t26x_status report(t26x_status status, t26x_error *error) {
    if (error) {
        error->code = status;
        std::snprintf(error->message, sizeof error->message, "%s",
                      status == T26X_OK ? "" : "injected transport/lifecycle failure");
    }
    return status;
}

template<class Action> void throws_status(Action action, t26x_status expected) {
    bool threw = false;
    try { action(); }
    catch (const t26x::error &error) {
        threw = true;
        require(error.code() == expected, "C++ wrapper preserves failing C status");
    }
    require(threw, "C++ wrapper must report injected failure");
}

void test_failed_close_consumes_owner() {
    new_scenario();
    {
        t26x::device camera;
        camera.open();
        close_result = T26X_IO;
        throws_status([&] { camera.close(); }, T26X_IO);
        require(!camera.is_open() && !camera.native_handle(), "failed USB stop still clears consumed C++ handle");
        require(!alive && closes == 1, "C close consumed the handle once");
        camera.close();
        require(closes == 1, "closing consumed owner again must be a no-op");
    }
    require(closes == 1 && duplicate_closes == 0, "destructor must not close a handle consumed on error");
}

void test_callback_rejection_retains_owner() {
    new_scenario();
    {
        t26x::device camera;
        camera.open();
        close_result = T26X_INVALID_STATE;
        throws_status([&] { camera.close(); }, T26X_INVALID_STATE);
        require(camera.is_open() && camera.native_handle() == &fake_handle,
                "callback rejection must preserve C++ ownership");
        require(alive && closes == 1, "callback rejection must preserve C handle");
        close_result = T26X_OK;
        camera.close();
        require(!camera.is_open() && !alive && closes == 2, "later legal close consumes rejected handle");
    }
    require(closes == 2 && duplicate_closes == 0, "destructor does not repeat successful later close");
}

void test_failed_reset_retains_closable_owner() {
    new_scenario();
    {
        t26x::device camera;
        camera.open();
        reset_result = T26X_IO;
        throws_status([&] { camera.reset(); }, T26X_IO);
        require(camera.is_open() && alive && resets == 1 && closes == 0,
                "failed reset retains handle for required cleanup");
        camera.close();
        require(!alive && !camera.is_open() && closes == 1, "failed reset handle remains closable");
    }
    require(closes == 1 && duplicate_closes == 0, "reset failure cleanup closes only once");
}

void test_successful_reset_and_destructor() {
    new_scenario();
    {
        t26x::device camera;
        camera.open();
        camera.reset();
        require(resets == 1 && closes == 1 && !alive && !camera.is_open(),
                "successful C++ reset closes its invalidated handle");
    }
    require(closes == 1 && duplicate_closes == 0, "reset leaves nothing for destructor to close");
    new_scenario();
    {
        t26x::device camera;
        camera.open();
        close_result = T26X_IO;
        // Destructor must consume the handle without throwing on USB failure.
    }
    require(!alive && closes == 1 && duplicate_closes == 0, "destructor releases handle once despite USB stop error");
}
} // namespace

extern "C" {
void t26x_open_options_init(t26x_open_options *options) {
    if (options) { *options = {}; options->boot_timeout_ms = 15000; }
}

const char *t26x_status_string(t26x_status) { return "injected status"; }

t26x_status t26x_open(const t26x_open_options *, t26x_device **output, t26x_error *error) {
    if (!output) return report(T26X_INVALID_ARGUMENT, error);
    if (alive) return report(T26X_BUSY, error);
    alive = true;
    *output = &fake_handle;
    return report(T26X_OK, error);
}

t26x_status t26x_close(t26x_device *device, t26x_error *error) {
    if (!device) return report(T26X_OK, error);
    ++closes;
    if (device != &fake_handle || !alive) {
        ++duplicate_closes;
        return report(T26X_INVALID_ARGUMENT, error);
    }
    if (close_result != T26X_INVALID_STATE) alive = false;
    return report(close_result, error);
}

t26x_status t26x_reset(t26x_device *device, t26x_error *error) {
    if (device != &fake_handle || !alive) return report(T26X_INVALID_ARGUMENT, error);
    ++resets;
    return report(reset_result, error);
}
} // extern "C"

int main() {
    try {
        test_failed_close_consumes_owner();
        test_callback_rejection_retains_owner();
        test_failed_reset_retains_closable_owner();
        test_successful_reset_and_destructor();
        std::printf("Ownership: %u checks passed.\n", checks);
        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::fprintf(stderr, "Ownership check failed: %s\n", error.what());
        return EXIT_FAILURE;
    }
}
