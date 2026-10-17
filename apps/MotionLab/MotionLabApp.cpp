#include "apps/MotionLab/MotionLabApp.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "libs/VehicleSimCore/physics/Integrator2D.h"

namespace motion_lab {

namespace {
    constexpr std::array<vehicle_sim::terrain::MaterialType, 7> kAvailableTerrains {
        { vehicle_sim::terrain::MaterialType::Tarmac, vehicle_sim::terrain::MaterialType::Ice,
            vehicle_sim::terrain::MaterialType::Sand, vehicle_sim::terrain::MaterialType::Mud,
            vehicle_sim::terrain::MaterialType::Grass, vehicle_sim::terrain::MaterialType::Gravel,
            vehicle_sim::terrain::MaterialType::Snow }
    };

    constexpr std::array<MotionLabVehicleType, 12> kAvailableVehicles { { MotionLabVehicleType::ClassicCar,
        MotionLabVehicleType::RaceCar, MotionLabVehicleType::Rocket, MotionLabVehicleType::Helicopter,
        MotionLabVehicleType::Hovercraft, MotionLabVehicleType::TrackedTank, MotionLabVehicleType::AntiGravity,
        MotionLabVehicleType::DriftCar, MotionLabVehicleType::MonsterTruck, MotionLabVehicleType::SemiTruck,
        MotionLabVehicleType::Forklift, MotionLabVehicleType::QuadcopterDrone } };
} // namespace

MotionLabApp::MotionLabApp()
    : MotionLabApp(MotionLabAppConfig { })
{
}

MotionLabApp::MotionLabApp(const MotionLabAppConfig& config)
    : mConfig { config }
{
    vehicle_sim::physics::RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1666.667;
    bodyConfig.position = vehicle_sim::math::Vec2 { 0.0, 0.0 };
    bodyConfig.orientation = 0.0;

    mBody = vehicle_sim::physics::RigidBody2D { bodyConfig };

    CameraConfig camConfig { };
    camConfig.viewportWidth = config.windowWidth;
    camConfig.viewportHeight = config.windowHeight;
    camConfig.baseZoom = 28.0;
    mCamera = MotionLabCamera { camConfig };

    applyVehicleProperties();
}

MotionLabApp::~MotionLabApp() noexcept
{
    if (mRenderer != nullptr) {
        SDL_DestroyRenderer(mRenderer);
        mRenderer = nullptr;
    }
    if (mWindow != nullptr) {
        SDL_DestroyWindow(mWindow);
        mWindow = nullptr;
    }
    if (!mConfig.headless) {
        SDL_Quit();
    }
}

bool MotionLabApp::initialize()
{
    if (mConfig.headless) {
        mRunning = true;
        resetSimulation();
        return true;
    }

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        return false;
    }

    mWindow = SDL_CreateWindow(mConfig.windowTitle.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        mConfig.windowWidth, mConfig.windowHeight, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

    if (mWindow == nullptr) {
        return false;
    }

    mRenderer = SDL_CreateRenderer(mWindow, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    if (mRenderer == nullptr) {
        return false;
    }

    mRunning = true;
    resetSimulation();
    return true;
}

void MotionLabApp::applyVehicleProperties() noexcept
{
    switch (mVehicleType) {
    case MotionLabVehicleType::ClassicCar:
        mBody.setMass(1000.0);
        mBody.setInertia(1666.667);
        mCar.setSurfaceMaterial(getSurfaceMaterial());
        break;
    case MotionLabVehicleType::RaceCar:
        mBody.setMass(mRaceCar.getConfig().mass);
        mBody.setInertia(mRaceCar.getConfig().inertia);
        break;
    case MotionLabVehicleType::Rocket:
        mBody.setMass(mRocket.getTotalMass());
        mBody.setInertia(mRocket.getInertia());
        break;
    case MotionLabVehicleType::Helicopter:
        mBody.setMass(mHelicopter.getConfig().mass);
        mBody.setInertia(4800.0);
        break;
    case MotionLabVehicleType::Hovercraft:
        mBody.setMass(mHovercraft.getConfig().mass);
        mBody.setInertia(2084.0);
        mHovercraft.setSurfaceFriction(getSurfaceMaterial().friction);
        break;
    case MotionLabVehicleType::TrackedTank:
        mBody.setMass(mTank.getConfig().mass);
        mBody.setInertia(30000.0);
        mTank.setSurfaceFriction(getSurfaceMaterial().friction);
        break;
    case MotionLabVehicleType::AntiGravity:
        mBody.setMass(mAntiGravity.getConfig().mass);
        mBody.setInertia(950.0);
        break;
    case MotionLabVehicleType::DriftCar:
        mBody.setMass(mDriftCar.getConfig().mass);
        mBody.setInertia(mDriftCar.getConfig().inertia);
        break;
    case MotionLabVehicleType::MonsterTruck:
        mBody.setMass(mMonsterCar.getConfig().mass);
        mBody.setInertia(12000.0);
        break;
    case MotionLabVehicleType::SemiTruck:
        mBody.setMass(mArticulatedTruck.getConfig().tractorMass + mArticulatedTruck.getConfig().trailerMass);
        mBody.setInertia(52000.0);
        break;
    case MotionLabVehicleType::Forklift:
        mBody.setMass(mForklift.getConfig().chassisMass);
        mBody.setInertia(mForklift.getConfig().chassisInertia);
        break;
    case MotionLabVehicleType::QuadcopterDrone:
        mBody.setMass(mDrone.getConfig().mass);
        mBody.setInertia(mDrone.getConfig().inertia);
        break;
    }

    double vehWidth { 2.0 };
    double vehLength { 4.0 };
    switch (mVehicleType) {
    case MotionLabVehicleType::ClassicCar:
        vehWidth = 2.00;
        vehLength = 4.0;
        break;
    case MotionLabVehicleType::RaceCar:
        vehWidth = 1.80;
        vehLength = 4.5;
        break;
    case MotionLabVehicleType::Rocket:
        vehWidth = 1.60;
        vehLength = 6.0;
        break;
    case MotionLabVehicleType::Helicopter:
        vehWidth = 2.40;
        vehLength = 6.0;
        break;
    case MotionLabVehicleType::Hovercraft:
        vehWidth = 2.20;
        vehLength = 4.0;
        break;
    case MotionLabVehicleType::TrackedTank:
        vehWidth = 2.40;
        vehLength = 5.0;
        break;
    case MotionLabVehicleType::AntiGravity:
        vehWidth = 2.40;
        vehLength = 4.8;
        break;
    case MotionLabVehicleType::DriftCar:
        vehWidth = 1.85;
        vehLength = 4.4;
        break;
    case MotionLabVehicleType::MonsterTruck:
        vehWidth = 3.00;
        vehLength = 5.2;
        break;
    case MotionLabVehicleType::SemiTruck:
        vehWidth = 2.50;
        vehLength = 12.0;
        break;
    case MotionLabVehicleType::Forklift:
        vehWidth = 1.30;
        vehLength = 2.8;
        break;
    case MotionLabVehicleType::QuadcopterDrone:
        vehWidth = 1.20;
        vehLength = 1.2;
        break;
    }

    mMooseCoordinator.updateDimensions(vehWidth, vehLength);
    mSlalomCoordinator.updateDimensions(vehWidth, vehLength);
    mSkidpadCoordinator.updateDimensions(vehWidth, vehLength);
    mCircuitCoordinator.updateDimensions(vehWidth, vehLength);
    mTireThermalCoordinator.setVehicleType(mVehicleType);
}

void MotionLabApp::resetSimulation() noexcept
{
    if (mCircuitCoordinator.isEnabled()) {
        mCircuitCoordinator.resetToGrid(mBody);
    } else if (mSlalomCoordinator.isEnabled()) {
        mSlalomCoordinator.resetCourse(mBody);
    } else if (mMooseCoordinator.isEnabled()) {
        mMooseCoordinator.resetCourse(mBody);
    } else if (mSkidpadCoordinator.isEnabled()) {
        mSkidpadCoordinator.resetToStaging(mBody);
    } else {
        mBody.setPosition(vehicle_sim::math::Vec2::zero());
        mBody.setLinearVelocity(vehicle_sim::math::Vec2::zero());
        mBody.setOrientation(0.0);
        mBody.setAngularVelocity(0.0);
        mBody.clearForces();
    }

    mCar.reset();
    mRaceCar.reset();
    mRocket.reset();
    mHelicopter.reset();
    mHovercraft.reset();
    mTank.reset();
    mAntiGravity.reset();
    mDriftCar.reset();
    mMonsterCar.reset();
    mArticulatedTruck.reset();
    mForklift.reset();
    mDrone.reset();
    mTireThermalCoordinator.reset();
    mTireSmokeBuffer.clear();

    applyVehicleProperties();

    mCamera.reset(mBody.getPosition());
    mSkids.clear();
    mTrail.clear();
    mAnimTimer = 0.0;

    mRearLeftState = RearTireState { };
    mRearRightState = RearTireState { };

    mKeyW = false;
    mKeyS = false;
    mKeyA = false;
    mKeyD = false;
    mKeySpace = false;
    mKeyQ = false;
    mKeyE = false;
}

void MotionLabApp::stop() noexcept
{
    mRunning = false;
}

void MotionLabApp::togglePause() noexcept
{
    mPaused = !mPaused;
}

void MotionLabApp::toggleBarrier() noexcept
{
    mBarrier = !mBarrier;
    mCamera.setFixedCamera(mBarrier);
}

void MotionLabApp::toggleGravity() noexcept
{
    mGravity = !mGravity;
}

void MotionLabApp::toggleDeadZone() noexcept
{
    mCamera.setDeadZoneEnabled(!mCamera.isDeadZoneEnabled());
}

void MotionLabApp::toggleSpeedZoom() noexcept
{
    mCamera.setSpeedZoomEnabled(!mCamera.isSpeedZoomEnabled());
}

void MotionLabApp::toggleInputHud() noexcept
{
    mInputHudVisible = !mInputHudVisible;
}

void MotionLabApp::toggleFullscreen() noexcept
{
    if (mWindow == nullptr) {
        return;
    }
    mIsFullscreen = !mIsFullscreen;
    SDL_SetWindowFullscreen(mWindow, mIsFullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
}

void MotionLabApp::toggleCircuit() noexcept
{
    mCircuitCoordinator.toggleEnabled();
    if (mCircuitCoordinator.isEnabled()) {
        if (mSlalomCoordinator.isEnabled()) {
            mSlalomCoordinator.setEnabled(false);
        }
        if (mMooseCoordinator.isEnabled()) {
            mMooseCoordinator.setEnabled(false);
        }
        if (mSkidpadCoordinator.isEnabled()) {
            mSkidpadCoordinator.setEnabled(false);
        }
        mCircuitCoordinator.resetToGrid(mBody);
        mCamera.reset(mBody.getPosition());
    }
}

void MotionLabApp::regenerateCircuit() noexcept
{
    mCircuitSeed += 137U;
    mCircuitCoordinator.regenerate(mCircuitSeed);
    if (mCircuitCoordinator.isEnabled()) {
        mCircuitCoordinator.resetToGrid(mBody);
        mCamera.reset(mBody.getPosition());
    }
}

bool MotionLabApp::isCircuitActive() const noexcept
{
    return mCircuitCoordinator.isEnabled();
}

const CircuitTrackCoordinator& MotionLabApp::getCircuit() const noexcept
{
    return mCircuitCoordinator;
}

CircuitTrackCoordinator& MotionLabApp::getCircuit() noexcept
{
    return mCircuitCoordinator;
}

void MotionLabApp::toggleSlalom() noexcept
{
    mSlalomCoordinator.toggleEnabled();
    if (mSlalomCoordinator.isEnabled()) {
        if (mCircuitCoordinator.isEnabled()) {
            mCircuitCoordinator.setEnabled(false);
        }
        if (mMooseCoordinator.isEnabled()) {
            mMooseCoordinator.setEnabled(false);
        }
        if (mSkidpadCoordinator.isEnabled()) {
            mSkidpadCoordinator.setEnabled(false);
        }
        mSlalomCoordinator.resetCourse(mBody);
        mCamera.reset(mBody.getPosition());
    }
}

void MotionLabApp::setSlalom(bool enabled) noexcept
{
    mSlalomCoordinator.setEnabled(enabled);
    if (mSlalomCoordinator.isEnabled()) {
        if (mCircuitCoordinator.isEnabled()) {
            mCircuitCoordinator.setEnabled(false);
        }
        if (mMooseCoordinator.isEnabled()) {
            mMooseCoordinator.setEnabled(false);
        }
        if (mSkidpadCoordinator.isEnabled()) {
            mSkidpadCoordinator.setEnabled(false);
        }
        mSlalomCoordinator.resetCourse(mBody);
        mCamera.reset(mBody.getPosition());
    }
}

bool MotionLabApp::isSlalomActive() const noexcept
{
    return mSlalomCoordinator.isEnabled();
}

const SlalomCourseCoordinator& MotionLabApp::getSlalom() const noexcept
{
    return mSlalomCoordinator;
}

SlalomCourseCoordinator& MotionLabApp::getSlalom() noexcept
{
    return mSlalomCoordinator;
}

void MotionLabApp::toggleMooseTest() noexcept
{
    mMooseCoordinator.toggleEnabled();
    if (mMooseCoordinator.isEnabled()) {
        if (mCircuitCoordinator.isEnabled()) {
            mCircuitCoordinator.setEnabled(false);
        }
        if (mSlalomCoordinator.isEnabled()) {
            mSlalomCoordinator.setEnabled(false);
        }
        if (mSkidpadCoordinator.isEnabled()) {
            mSkidpadCoordinator.setEnabled(false);
        }
        mMooseCoordinator.resetCourse(mBody);
        mCamera.reset(mBody.getPosition());
    }
}

void MotionLabApp::setMooseTest(bool enabled) noexcept
{
    mMooseCoordinator.setEnabled(enabled);
    if (mMooseCoordinator.isEnabled()) {
        if (mCircuitCoordinator.isEnabled()) {
            mCircuitCoordinator.setEnabled(false);
        }
        if (mSlalomCoordinator.isEnabled()) {
            mSlalomCoordinator.setEnabled(false);
        }
        if (mSkidpadCoordinator.isEnabled()) {
            mSkidpadCoordinator.setEnabled(false);
        }
        mMooseCoordinator.resetCourse(mBody);
        mCamera.reset(mBody.getPosition());
    }
}

bool MotionLabApp::isMooseTestActive() const noexcept
{
    return mMooseCoordinator.isEnabled();
}

const MooseTestCoordinator& MotionLabApp::getMooseTest() const noexcept
{
    return mMooseCoordinator;
}

MooseTestCoordinator& MotionLabApp::getMooseTest() noexcept
{
    return mMooseCoordinator;
}

void MotionLabApp::toggleSkidpad() noexcept
{
    mSkidpadCoordinator.toggleEnabled();
    if (mSkidpadCoordinator.isEnabled()) {
        if (mCircuitCoordinator.isEnabled()) {
            mCircuitCoordinator.setEnabled(false);
        }
        if (mSlalomCoordinator.isEnabled()) {
            mSlalomCoordinator.setEnabled(false);
        }
        if (mMooseCoordinator.isEnabled()) {
            mMooseCoordinator.setEnabled(false);
        }
        mSkidpadCoordinator.resetToStaging(mBody);
        mCamera.reset(mBody.getPosition());
    }
}

void MotionLabApp::setSkidpad(bool enabled) noexcept
{
    mSkidpadCoordinator.setEnabled(enabled);
    if (mSkidpadCoordinator.isEnabled()) {
        if (mCircuitCoordinator.isEnabled()) {
            mCircuitCoordinator.setEnabled(false);
        }
        if (mSlalomCoordinator.isEnabled()) {
            mSlalomCoordinator.setEnabled(false);
        }
        if (mMooseCoordinator.isEnabled()) {
            mMooseCoordinator.setEnabled(false);
        }
        mSkidpadCoordinator.resetToStaging(mBody);
        mCamera.reset(mBody.getPosition());
    }
}

void MotionLabApp::setSkidpadRadius(SkidpadRadiusPreset preset) noexcept
{
    mSkidpadCoordinator.setRadiusPreset(preset);
    if (mSkidpadCoordinator.isEnabled()) {
        mSkidpadCoordinator.resetToStaging(mBody);
        mCamera.reset(mBody.getPosition());
    }
}

bool MotionLabApp::isSkidpadActive() const noexcept
{
    return mSkidpadCoordinator.isEnabled();
}

const SkidpadCoordinator& MotionLabApp::getSkidpad() const noexcept
{
    return mSkidpadCoordinator;
}

SkidpadCoordinator& MotionLabApp::getSkidpad() noexcept
{
    return mSkidpadCoordinator;
}

void MotionLabApp::cycleTerrain() noexcept
{
    mTerrainIndex = (mTerrainIndex + 1U) % kAvailableTerrains.size();
    setTerrain(kAvailableTerrains[mTerrainIndex]);
}

void MotionLabApp::setTerrain(vehicle_sim::terrain::MaterialType type) noexcept
{
    for (std::size_t i { 0U }; i < kAvailableTerrains.size(); ++i) {
        if (kAvailableTerrains[i] == type) {
            mTerrainIndex = i;
            break;
        }
    }
    const auto mat = vehicle_sim::terrain::SurfaceMaterial::getPreset(type);
    mCar.setSurfaceMaterial(mat);
    mHovercraft.setSurfaceFriction(mat.friction);
    mTank.setSurfaceFriction(mat.friction);
}

vehicle_sim::terrain::MaterialType MotionLabApp::getTerrain() const noexcept
{
    return mCar.getSurfaceMaterial().type;
}

const vehicle_sim::terrain::SurfaceMaterial& MotionLabApp::getSurfaceMaterial() const noexcept
{
    return mCar.getSurfaceMaterial();
}

void MotionLabApp::cycleVehicle() noexcept
{
    mVehicleIndex = (mVehicleIndex + 1U) % kAvailableVehicles.size();
    setVehicleType(kAvailableVehicles[mVehicleIndex]);
}

void MotionLabApp::setVehicleType(MotionLabVehicleType type) noexcept
{
    mVehicleType = type;
    for (std::size_t i { 0U }; i < kAvailableVehicles.size(); ++i) {
        if (kAvailableVehicles[i] == type) {
            mVehicleIndex = i;
            break;
        }
    }
    applyVehicleProperties();
}

MotionLabVehicleType MotionLabApp::getVehicleType() const noexcept
{
    return mVehicleType;
}

std::string_view MotionLabApp::getVehicleName() const noexcept
{
    return getVehicleTypeName(mVehicleType);
}

vehicle_sim::math::Vec2 MotionLabApp::computeSupportPoint(const vehicle_sim::math::Vec2& normal) const noexcept
{
    double halfLen { 2.0 };
    double halfWid { 1.0 };

    switch (mVehicleType) {
    case MotionLabVehicleType::ClassicCar:
        return mCar.computeSupportPoint(mBody, normal);
    case MotionLabVehicleType::RaceCar:
        halfLen = 2.25;
        halfWid = 0.90;
        break;
    case MotionLabVehicleType::Rocket:
        halfLen = 3.00;
        halfWid = 0.80;
        break;
    case MotionLabVehicleType::Helicopter:
        halfLen = 3.00;
        halfWid = 1.20;
        break;
    case MotionLabVehicleType::Hovercraft:
        halfLen = 2.00;
        halfWid = 1.10;
        break;
    case MotionLabVehicleType::TrackedTank:
        halfLen = 2.50;
        halfWid = 1.20;
        break;
    case MotionLabVehicleType::AntiGravity:
        halfLen = 2.40;
        halfWid = 1.20;
        break;
    case MotionLabVehicleType::DriftCar:
        halfLen = 2.20;
        halfWid = 0.925;
        break;
    case MotionLabVehicleType::MonsterTruck:
        halfLen = 2.60;
        halfWid = 1.50;
        break;
    case MotionLabVehicleType::SemiTruck:
        halfLen = 6.00;
        halfWid = 1.25;
        break;
    case MotionLabVehicleType::Forklift:
        halfLen = 1.40;
        halfWid = 0.65;
        break;
    case MotionLabVehicleType::QuadcopterDrone:
        halfLen = 0.60;
        halfWid = 0.60;
        break;
    }

    const std::array<vehicle_sim::math::Vec2, 4> points { { mBody.toWorldPoint(
                                                                vehicle_sim::math::Vec2 { -halfLen, -halfWid }),
        mBody.toWorldPoint(vehicle_sim::math::Vec2 { halfLen, -halfWid }),
        mBody.toWorldPoint(vehicle_sim::math::Vec2 { halfLen, halfWid }),
        mBody.toWorldPoint(vehicle_sim::math::Vec2 { -halfLen, halfWid }) } };

    double minProj { points[0].dot(normal) };
    for (std::size_t i { 1 }; i < points.size(); ++i) {
        minProj = std::min(minProj, points[i].dot(normal));
    }

    vehicle_sim::math::Vec2 sum { 0.0, 0.0 };
    double count { 0.0 };
    for (const auto& pt : points) {
        if (std::abs(pt.dot(normal) - minProj) < 1e-8) {
            sum += pt;
            count += 1.0;
        }
    }
    return (count > 0.0) ? (sum / count) : points[0];
}

void MotionLabApp::processEvents() noexcept
{
    SDL_Event event { };
    while (SDL_PollEvent(&event) != 0) {
        if (event.type == SDL_QUIT) {
            mRunning = false;
        } else if (event.type == SDL_WINDOWEVENT) {
            if (event.window.event == SDL_WINDOWEVENT_RESIZED || event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                mConfig.windowWidth = event.window.data1;
                mConfig.windowHeight = event.window.data2;
                mCamera.resize(mConfig.windowWidth, mConfig.windowHeight);
            }
        } else if (event.type == SDL_KEYDOWN) {
            const SDL_Keycode key { event.key.keysym.sym };
            if (!event.key.repeat) {
                switch (key) {
                case SDLK_w:
                case SDLK_UP:
                    mKeyW = true;
                    break;
                case SDLK_s:
                case SDLK_DOWN:
                    mKeyS = true;
                    break;
                case SDLK_a:
                case SDLK_LEFT:
                    mKeyA = true;
                    break;
                case SDLK_d:
                case SDLK_RIGHT:
                    mKeyD = true;
                    break;
                case SDLK_SPACE:
                    mKeySpace = true;
                    break;
                case SDLK_r:
                    resetSimulation();
                    break;
                case SDLK_p:
                    togglePause();
                    break;
                case SDLK_b:
                    toggleBarrier();
                    break;
                case SDLK_g:
                    if ((event.key.keysym.mod & KMOD_SHIFT) != 0) {
                        mWindCoordinator.triggerDiscreteGust(12.0, 2.5);
                    } else {
                        toggleGravity();
                    }
                    break;
                case SDLK_z:
                    toggleDeadZone();
                    break;
                case SDLK_x:
                    toggleSpeedZoom();
                    break;
                case SDLK_h:
                    if ((event.key.keysym.mod & KMOD_SHIFT) != 0) {
                        toggleWindHud();
                    } else {
                        toggleAdvancedHud();
                    }
                    break;
                case SDLK_TAB:
                    toggleAllHud();
                    break;
                case SDLK_i:
                    toggleInputHud();
                    break;
                case SDLK_f:
                    toggleFullscreen();
                    break;
                case SDLK_c:
                    toggleCircuit();
                    break;
                case SDLK_l:
                    toggleSlalom();
                    break;
                case SDLK_m:
                    toggleMooseTest();
                    break;
                case SDLK_k:
                    toggleSkidpad();
                    break;
                case SDLK_n:
                    regenerateCircuit();
                    break;
                case SDLK_t:
                    if ((event.key.keysym.mod & KMOD_CTRL) != 0) {
                        resetTireThermalStates();
                    } else if ((event.key.keysym.mod & KMOD_SHIFT) != 0) {
                        toggleTireThermalHud();
                    } else {
                        cycleTerrain();
                    }
                    break;
                case SDLK_y:
                    toggleTireThermalHud();
                    break;
                case SDLK_v:
                    cycleVehicle();
                    break;
                case SDLK_u:
                    mWindCoordinator.cycleNextPreset();
                    break;
                case SDLK_o:
                    mWindRenderer.toggleOverlay();
                    break;
                case SDLK_LEFTBRACKET:
                    if ((event.key.keysym.mod & KMOD_SHIFT) != 0) {
                        mWindCoordinator.rotateWindHeading(-15.0);
                    } else {
                        mWindCoordinator.adjustWindSpeed(0.80);
                    }
                    break;
                case SDLK_RIGHTBRACKET:
                    if ((event.key.keysym.mod & KMOD_SHIFT) != 0) {
                        mWindCoordinator.rotateWindHeading(15.0);
                    } else {
                        mWindCoordinator.adjustWindSpeed(1.25);
                    }
                    break;
                case SDLK_q:
                    mKeyQ = true;
                    break;
                case SDLK_e:
                    if ((event.key.keysym.mod & KMOD_CTRL) != 0) {
                        toggleRecording();
                    } else {
                        mKeyE = true;
                    }
                    break;
                case SDLK_F1:
                    setVehicleType(MotionLabVehicleType::ClassicCar);
                    break;
                case SDLK_F2:
                    setVehicleType(MotionLabVehicleType::RaceCar);
                    break;
                case SDLK_F3:
                    setVehicleType(MotionLabVehicleType::Rocket);
                    break;
                case SDLK_F4:
                    setVehicleType(MotionLabVehicleType::Helicopter);
                    break;
                case SDLK_F5:
                    setVehicleType(MotionLabVehicleType::Hovercraft);
                    break;
                case SDLK_F6:
                    setVehicleType(MotionLabVehicleType::TrackedTank);
                    break;
                case SDLK_F7:
                    if ((event.key.keysym.mod & KMOD_SHIFT) != 0) {
                        mWindCoordinator.toggleWindActive();
                    } else if ((event.key.keysym.mod & KMOD_CTRL) != 0) {
                        mWindCoordinator.cycleNextPreset();
                    } else {
                        setVehicleType(MotionLabVehicleType::AntiGravity);
                    }
                    break;
                case SDLK_F8:
                    setVehicleType(MotionLabVehicleType::DriftCar);
                    break;
                case SDLK_F9:
                    if ((event.key.keysym.mod & KMOD_CTRL) != 0) {
                        toggleRecording();
                    } else {
                        setVehicleType(MotionLabVehicleType::MonsterTruck);
                    }
                    break;
                case SDLK_F10:
                    setVehicleType(MotionLabVehicleType::SemiTruck);
                    break;
                case SDLK_F11:
                    setVehicleType(MotionLabVehicleType::Forklift);
                    break;
                case SDLK_F12:
                    setVehicleType(MotionLabVehicleType::QuadcopterDrone);
                    break;
                case SDLK_1:
                case SDLK_KP_1:
                    if (mSkidpadCoordinator.isEnabled()) {
                        setSkidpadRadius(SkidpadRadiusPreset::Radius15m);
                    } else {
                        setTerrain(vehicle_sim::terrain::MaterialType::Tarmac);
                    }
                    break;
                case SDLK_2:
                case SDLK_KP_2:
                    if (mSkidpadCoordinator.isEnabled()) {
                        setSkidpadRadius(SkidpadRadiusPreset::Radius30m);
                    } else {
                        setTerrain(vehicle_sim::terrain::MaterialType::Ice);
                    }
                    break;
                case SDLK_3:
                case SDLK_KP_3:
                    setTerrain(vehicle_sim::terrain::MaterialType::Sand);
                    break;
                case SDLK_4:
                case SDLK_KP_4:
                    setTerrain(vehicle_sim::terrain::MaterialType::Mud);
                    break;
                case SDLK_5:
                case SDLK_KP_5:
                    setTerrain(vehicle_sim::terrain::MaterialType::Grass);
                    break;
                case SDLK_6:
                case SDLK_KP_6:
                    setTerrain(vehicle_sim::terrain::MaterialType::Gravel);
                    break;
                case SDLK_7:
                case SDLK_KP_7:
                    setTerrain(vehicle_sim::terrain::MaterialType::Snow);
                    break;
                case SDLK_ESCAPE:
                    mRunning = false;
                    break;
                default:
                    break;
                }
            }
        } else if (event.type == SDL_KEYUP) {
            const SDL_Keycode key { event.key.keysym.sym };
            switch (key) {
            case SDLK_w:
            case SDLK_UP:
                mKeyW = false;
                break;
            case SDLK_s:
            case SDLK_DOWN:
                mKeyS = false;
                break;
            case SDLK_a:
            case SDLK_LEFT:
                mKeyA = false;
                break;
            case SDLK_d:
            case SDLK_RIGHT:
                mKeyD = false;
                break;
            case SDLK_SPACE:
                mKeySpace = false;
                break;
            case SDLK_q:
                mKeyQ = false;
                break;
            case SDLK_e:
                mKeyE = false;
                break;
            default:
                break;
            }
        }
    }
}

void MotionLabApp::updateTireSkidmarks(double dt) noexcept
{
    mSkids.update(dt);

    if (mVehicleType == MotionLabVehicleType::ClassicCar) {
        const auto& tires = mCar.getTires();
        const auto& rearLeft = tires[vehicle_sim::dynamics::MotionLabCarDynamics::kRearLeft];
        const auto& rearRight = tires[vehicle_sim::dynamics::MotionLabCarDynamics::kRearRight];

        if (rearLeft.skidding && mRearLeftState.skidding) {
            mSkids.addSegment(mRearLeftState.position, rearLeft.worldPosition);
        }
        mRearLeftState.position = rearLeft.worldPosition;
        mRearLeftState.skidding = rearLeft.skidding;

        if (rearRight.skidding && mRearRightState.skidding) {
            mSkids.addSegment(mRearRightState.position, rearRight.worldPosition);
        }
        mRearRightState.position = rearRight.worldPosition;
        mRearRightState.skidding = rearRight.skidding;
    } else if (mVehicleType == MotionLabVehicleType::RaceCar) {
        const auto& wheels = mRaceCar.getWheelStates();
        const auto worldRL = mBody.toWorldPoint(wheels[2].localOffset);
        const auto worldRR = mBody.toWorldPoint(wheels[3].localOffset);
        const bool skiddingRL { std::abs(wheels[2].slipAngle) > 0.15 || std::abs(wheels[2].slipRatio) > 0.15 };
        const bool skiddingRR { std::abs(wheels[3].slipAngle) > 0.15 || std::abs(wheels[3].slipRatio) > 0.15 };

        if (skiddingRL && mRearLeftState.skidding) {
            mSkids.addSegment(mRearLeftState.position, worldRL);
        }
        mRearLeftState.position = worldRL;
        mRearLeftState.skidding = skiddingRL;

        if (skiddingRR && mRearRightState.skidding) {
            mSkids.addSegment(mRearRightState.position, worldRR);
        }
        mRearRightState.position = worldRR;
        mRearRightState.skidding = skiddingRR;
    } else if (mVehicleType == MotionLabVehicleType::TrackedTank) {
        const auto worldL = mBody.toWorldPoint({ -1.8, 1.05 });
        const auto worldR = mBody.toWorldPoint({ -1.8, -1.05 });
        const bool skidding { std::abs(mBody.getAngularVelocity()) > 0.25 };

        if (skidding && mRearLeftState.skidding) {
            mSkids.addSegment(mRearLeftState.position, worldL);
        }
        mRearLeftState.position = worldL;
        mRearLeftState.skidding = skidding;

        if (skidding && mRearRightState.skidding) {
            mSkids.addSegment(mRearRightState.position, worldR);
        }
        mRearRightState.position = worldR;
        mRearRightState.skidding = skidding;
    } else if (mVehicleType == MotionLabVehicleType::DriftCar) {
        const auto& wheels = mDriftCar.getWheelStates();
        const auto worldRL = mBody.toWorldPoint(wheels[2].localOffset);
        const auto worldRR = mBody.toWorldPoint(wheels[3].localOffset);
        const bool skiddingRL { std::abs(wheels[2].slipAngle) > 0.12 || mDriftCar.isDrifting() };
        const bool skiddingRR { std::abs(wheels[3].slipAngle) > 0.12 || mDriftCar.isDrifting() };

        if (skiddingRL && mRearLeftState.skidding) {
            mSkids.addSegment(mRearLeftState.position, worldRL);
        }
        mRearLeftState.position = worldRL;
        mRearLeftState.skidding = skiddingRL;

        if (skiddingRR && mRearRightState.skidding) {
            mSkids.addSegment(mRearRightState.position, worldRR);
        }
        mRearRightState.position = worldRR;
        mRearRightState.skidding = skiddingRR;
    } else if (mVehicleType == MotionLabVehicleType::MonsterTruck) {
        const auto worldRL = mBody.toWorldPoint({ -1.25, 1.15 });
        const auto worldRR = mBody.toWorldPoint({ -1.25, -1.15 });
        const bool skidding { std::abs(mBody.getAngularVelocity()) > 0.35 || mKeySpace };

        if (skidding && mRearLeftState.skidding) {
            mSkids.addSegment(mRearLeftState.position, worldRL);
        }
        mRearLeftState.position = worldRL;
        mRearLeftState.skidding = skidding;

        if (skidding && mRearRightState.skidding) {
            mSkids.addSegment(mRearRightState.position, worldRR);
        }
        mRearRightState.position = worldRR;
        mRearRightState.skidding = skidding;
    } else if (mVehicleType == MotionLabVehicleType::SemiTruck) {
        const auto worldRL = mBody.toWorldPoint({ -2.0, 1.25 });
        const auto worldRR = mBody.toWorldPoint({ -2.0, -1.25 });
        const bool skidding { mArticulatedTruck.getTelemetry().isJackknifed
            || std::abs(mBody.getAngularVelocity()) > 0.30 };

        if (skidding && mRearLeftState.skidding) {
            mSkids.addSegment(mRearLeftState.position, worldRL);
        }
        mRearLeftState.position = worldRL;
        mRearLeftState.skidding = skidding;

        if (skidding && mRearRightState.skidding) {
            mSkids.addSegment(mRearRightState.position, worldRR);
        }
        mRearRightState.position = worldRR;
        mRearRightState.skidding = skidding;
    } else if (mVehicleType == MotionLabVehicleType::Forklift) {
        const auto worldFL = mBody.toWorldPoint({ 0.8, 0.55 });
        const auto worldFR = mBody.toWorldPoint({ 0.8, -0.55 });
        const bool skidding { std::abs(mBody.getAngularVelocity()) > 0.40 };

        if (skidding && mRearLeftState.skidding) {
            mSkids.addSegment(mRearLeftState.position, worldFL);
        }
        mRearLeftState.position = worldFL;
        mRearLeftState.skidding = skidding;

        if (skidding && mRearRightState.skidding) {
            mSkids.addSegment(mRearRightState.position, worldFR);
        }
        mRearRightState.position = worldFR;
        mRearRightState.skidding = skidding;
    }
}

void MotionLabApp::stepPhysics(double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    mAnimTimer += dt;

    switch (mVehicleType) {
    case MotionLabVehicleType::ClassicCar: {
        const double throttle { (mKeyW ? 1.0 : 0.0) - (mKeyS ? 1.0 : 0.0) };
        const double steer { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
        const bool handbrake { mKeySpace };
        mCar.setInput({ throttle, steer, handbrake });
        mCar.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::RaceCar: {
        const double throttle { mKeyW ? 1.0 : 0.0 };
        const double brake { mKeyS ? 1.0 : 0.0 };
        const double steer { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
        const bool drs { mKeySpace };
        mRaceCar.setThrottle(throttle);
        mRaceCar.setBrake(brake);
        mRaceCar.setSteer(steer);
        mRaceCar.setDrsActive(drs);
        mRaceCar.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::Rocket: {
        const double throttle { mKeyW ? 1.0 : 0.0 };
        const double gimbal { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
        const double rcsRot { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
        const double rcsTransX { (mKeySpace ? 1.0 : 0.0) - (mKeyS ? 1.0 : 0.0) };
        mRocket.setInput({ throttle, gimbal, rcsRot, { rcsTransX, 0.0 }, true });
        mRocket.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::Helicopter: {
        const double cyclic { (mKeyW ? 1.0 : 0.0) - (mKeyS ? 1.0 : 0.0) };
        const double rudder { (mKeyD ? 1.0 : 0.0) - (mKeyA ? 1.0 : 0.0) };
        const double collective { mKeySpace ? 1.0 : (mKeyW || mKeyS ? 0.8 : 0.68) };
        mHelicopter.setInput({ collective, cyclic, rudder, true, 10.0 });
        mHelicopter.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::Hovercraft: {
        const double throttle { (mKeyW ? 1.0 : 0.0) - (mKeyS ? 1.0 : 0.0) };
        const double steer { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
        const bool liftFan { !mKeySpace };
        mHovercraft.setInput({ throttle, steer, liftFan, 0.35 });
        mHovercraft.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::TrackedTank: {
        double leftTrack { 0.0 };
        double rightTrack { 0.0 };
        const double fwd { (mKeyW ? 1.0 : 0.0) - (mKeyS ? 1.0 : 0.0) };
        const double turn { (mKeyD ? 1.0 : 0.0) - (mKeyA ? 1.0 : 0.0) };

        if (std::abs(fwd) > 0.01) {
            leftTrack = fwd + turn * 0.4;
            rightTrack = fwd - turn * 0.4;
        } else {
            leftTrack = turn;
            rightTrack = -turn;
        }

        leftTrack = std::clamp(leftTrack, -1.0, 1.0);
        rightTrack = std::clamp(rightTrack, -1.0, 1.0);

        if (mKeySpace) {
            mTank.fireRecoil();
        }
        mTank.setInput({ leftTrack, rightTrack, mKeySpace, 0.0 });
        mTank.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::AntiGravity: {
        const double throttle { (mKeyW ? 1.0 : 0.0) - (mKeyS ? 1.0 : 0.0) };
        const double steer { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
        const double leftAirbrake { mKeyQ ? 1.0 : 0.0 };
        const double rightAirbrake { mKeyE ? 1.0 : 0.0 };
        const bool boost { mKeySpace };
        mAntiGravity.setInput({ throttle, steer, leftAirbrake, rightAirbrake, boost, 1.2 });
        mAntiGravity.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::DriftCar: {
        const double throttle { mKeyW ? 1.0 : 0.0 };
        const double brake { mKeyS ? 1.0 : 0.0 };
        const double steer { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
        const double handbrake { mKeySpace ? 1.0 : 0.0 };
        if (mKeyE) {
            mDriftCar.triggerClutchKick();
        }
        mDriftCar.setThrottle(throttle);
        mDriftCar.setBrake(brake);
        mDriftCar.setSteer(steer);
        mDriftCar.setHandbrake(handbrake);
        mDriftCar.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::MonsterTruck: {
        const double throttle { mKeyW ? 1.0 : 0.0 };
        const double brake { mKeyS ? 1.0 : 0.0 };
        const double steer { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
        const bool handbrake { mKeySpace };
        mMonsterCar.setInput({ throttle, steer, brake, 0.0, handbrake, false });
        mMonsterCar.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::SemiTruck: {
        const double throttle { (mKeyW ? 1.0 : 0.0) - (mKeyS ? 1.0 : 0.0) };
        const double steer { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
        const double serviceBrake { (mKeyS && mBody.getLinearVelocity().length() > 0.5) ? 1.0 : 0.0 };
        const double trailerBrake { mKeySpace ? 1.0 : 0.0 };
        mArticulatedTruck.setInput({ throttle, steer, serviceBrake, trailerBrake });
        mArticulatedTruck.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::Forklift: {
        const double throttle { mKeyW ? 1.0 : 0.0 };
        const double brake { mKeyS ? 1.0 : 0.0 };
        const double steer { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
        mForklift.setReverse(mKeySpace);
        if (mKeyQ) {
            mForklift.setMastHeight(std::min(4.5, mForklift.getCompositeCogHeight() + 0.05));
        } else if (mKeyE) {
            mForklift.setMastHeight(std::max(0.0, mForklift.getCompositeCogHeight() - 0.05));
        }
        mForklift.setThrottle(throttle);
        mForklift.setBrake(brake);
        mForklift.setSteer(steer);
        mForklift.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::QuadcopterDrone: {
        const double collective { mKeyW ? 1.0 : (mKeyS ? 0.2 : 0.6) };
        const double pitchCmd { ((mKeyS ? 1.0 : 0.0) - (mKeyW ? 1.0 : 0.0)) * 0.25 };
        mDrone.setArmed(true);
        mDrone.setThrottle(collective);
        mDrone.setPitchAngle(pitchCmd);
        mDrone.step(mBody, dt);
        break;
    }
    }

    // Optional downward vertical gravity and linear atmospheric terminal drag
    if (mGravity) {
        constexpr double kGravityAcc { 3.0 }; // m/s^2
        constexpr double kTerminalSpeed { 30.0 }; // m/s
        const double dragCoeff { (mBody.getMass() * kGravityAcc) / kTerminalSpeed };
        const auto v = mBody.getLinearVelocity();

        const vehicle_sim::math::Vec2 gravForce { -dragCoeff * v.x, -mBody.getMass() * kGravityAcc - dragCoeff * v.y };
        mBody.applyForce(gravForce);
    }

    mWindCoordinator.stepPhysics(mBody, mVehicleType, dt);

    if (mCircuitCoordinator.isEnabled()) {
        mCircuitCoordinator.stepPhysics(mBody, dt);
        const auto mat = mCircuitCoordinator.getActiveMaterial();
        mCar.setSurfaceMaterial(mat);
        mHovercraft.setSurfaceFriction(mat.friction);
        mTank.setSurfaceFriction(mat.friction);
    }

    if (mSlalomCoordinator.isEnabled()) {
        mSlalomCoordinator.stepPhysics(mBody, dt);
    }

    if (mMooseCoordinator.isEnabled()) {
        const double throttleInput = (mKeyW && !mKeyS) ? 1.0 : 0.0;
        const double brakeInput = mKeyS ? 1.0 : 0.0;
        mMooseCoordinator.stepPhysics(mBody, throttleInput, brakeInput, dt);
    }

    if (mSkidpadCoordinator.isEnabled()) {
        double steerAngle { 0.0 };
        double wheelbase { 2.6 };
        switch (mVehicleType) {
        case MotionLabVehicleType::ClassicCar:
            steerAngle = mCar.getTires()[vehicle_sim::dynamics::MotionLabCarDynamics::kFrontLeft].steerAngle;
            wheelbase = 2.6;
            break;
        case MotionLabVehicleType::RaceCar:
            steerAngle = ((mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0)) * 0.45;
            wheelbase = 2.8;
            break;
        case MotionLabVehicleType::Rocket:
            steerAngle = ((mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0)) * 0.35;
            wheelbase = 3.0;
            break;
        case MotionLabVehicleType::Helicopter:
            steerAngle = ((mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0)) * 0.30;
            wheelbase = 2.0;
            break;
        case MotionLabVehicleType::Hovercraft:
            steerAngle = ((mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0)) * 0.40;
            wheelbase = 2.2;
            break;
        case MotionLabVehicleType::TrackedTank:
            steerAngle = ((mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0)) * 0.30;
            wheelbase = 3.2;
            break;
        case MotionLabVehicleType::AntiGravity:
            steerAngle = ((mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0)) * 0.35;
            wheelbase = 2.8;
            break;
        case MotionLabVehicleType::DriftCar:
            steerAngle = mDriftCar.getWheelStates()[0].steerAngle;
            wheelbase = 2.55;
            break;
        case MotionLabVehicleType::MonsterTruck:
            steerAngle = mMonsterCar.getInput().steer * 0.50;
            wheelbase = 2.5;
            break;
        case MotionLabVehicleType::SemiTruck:
            steerAngle = ((mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0)) * 0.55;
            wheelbase = 3.8;
            break;
        case MotionLabVehicleType::Forklift:
            steerAngle = ((mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0)) * 1.45;
            wheelbase = 1.8;
            break;
        case MotionLabVehicleType::QuadcopterDrone:
            steerAngle = 0.0;
            wheelbase = 1.0;
            break;
        }
        mSkidpadCoordinator.stepPhysics(mBody, steerAngle, wheelbase, dt);
    }

    vehicle_sim::physics::Integrator2D::integrate(mBody, dt);

    if (mBarrier) {
        vehicle_sim::physics::BarrierSolver2D::solveInside(mBody, mCamera.getBounds(),
            [this](const vehicle_sim::math::Vec2& normal) noexcept { return computeSupportPoint(normal); });
    }

    mTrail.update(mBody.getPosition(), dt);
    updateTireSkidmarks(dt);
    updateTireThermals(dt);

    double wheelbase { 2.6 };
    double trackWidth { 1.5 };
    double cgHeight { 0.45 };
    double frontSlip { 0.0 };
    double rearSlip { 0.0 };

    switch (mVehicleType) {
    case MotionLabVehicleType::ClassicCar: {
        wheelbase = 2.6;
        trackWidth = 1.5;
        cgHeight = 0.45;
        const auto& flTire = mCar.getTires()[vehicle_sim::dynamics::MotionLabCarDynamics::kFrontLeft];
        const auto& rlTire = mCar.getTires()[vehicle_sim::dynamics::MotionLabCarDynamics::kRearLeft];
        frontSlip = std::atan2(flTire.lateralSpeed, std::max(0.1, std::abs(flTire.rollingSpeed)));
        rearSlip = std::atan2(rlTire.lateralSpeed, std::max(0.1, std::abs(rlTire.rollingSpeed)));
        break;
    }
    case MotionLabVehicleType::RaceCar:
        wheelbase = 2.8;
        trackWidth = 1.6;
        cgHeight = 0.32;
        break;
    case MotionLabVehicleType::Rocket:
        wheelbase = 3.0;
        trackWidth = 1.0;
        cgHeight = 0.50;
        break;
    case MotionLabVehicleType::Helicopter:
        wheelbase = 2.0;
        trackWidth = 1.4;
        cgHeight = 0.60;
        break;
    case MotionLabVehicleType::Hovercraft:
        wheelbase = 2.2;
        trackWidth = 1.5;
        cgHeight = 0.40;
        break;
    case MotionLabVehicleType::TrackedTank:
        wheelbase = 3.2;
        trackWidth = 2.2;
        cgHeight = 0.70;
        break;
    case MotionLabVehicleType::AntiGravity:
        wheelbase = 2.8;
        trackWidth = 1.6;
        cgHeight = 0.35;
        break;
    case MotionLabVehicleType::DriftCar:
        wheelbase = 2.55;
        trackWidth = 1.55;
        cgHeight = 0.42;
        frontSlip = mDriftCar.getWheelStates()[0].slipAngle;
        rearSlip = mDriftCar.getWheelStates()[2].slipAngle;
        break;
    case MotionLabVehicleType::MonsterTruck:
        wheelbase = 2.5;
        trackWidth = 2.1;
        cgHeight = 0.85;
        break;
    case MotionLabVehicleType::SemiTruck:
        wheelbase = 3.8;
        trackWidth = 2.0;
        cgHeight = 0.95;
        break;
    case MotionLabVehicleType::Forklift:
        wheelbase = 1.8;
        trackWidth = 1.1;
        cgHeight = mForklift.getCompositeCogHeight();
        break;
    case MotionLabVehicleType::QuadcopterDrone:
        wheelbase = 1.0;
        trackWidth = 1.0;
        cgHeight = 0.25;
        break;
    }

    const double surfaceMu { getSurfaceMaterial().friction };
    mTelemetryCoordinator.update(
        mBody, mBody.getMass(), wheelbase, trackWidth, cgHeight, surfaceMu, frontSlip, rearSlip, mAnimTimer, dt);
}

void MotionLabApp::renderFrame() noexcept
{
    if (mRenderer == nullptr) {
        return;
    }

    VehicleRenderContext context { };
    context.type = mVehicleType;
    context.terrain = getTerrain();
    context.car = &mCar;
    context.raceCar = &mRaceCar;
    context.rocket = &mRocket;
    context.helicopter = &mHelicopter;
    context.hovercraft = &mHovercraft;
    context.tank = &mTank;
    context.antiGravity = &mAntiGravity;
    context.driftCar = &mDriftCar;
    context.monsterCar = &mMonsterCar;
    context.articulatedTruck = &mArticulatedTruck;
    context.forklift = &mForklift;
    context.quadcopter = &mDrone;
    context.animTimer = mAnimTimer;

    mWireframe.renderScene(mRenderer, mCamera, context, mBody, mSkids, mTrail, mBarrier);
    mTireThermalRenderer.renderSmoke(mRenderer, mTireSmokeBuffer, mCamera);

    if (mCircuitCoordinator.isEnabled()) {
        CircuitTrackRenderer::renderTrack(mRenderer, mCamera, mCircuitCoordinator);
    }

    if (mSlalomCoordinator.isEnabled()) {
        SlalomCourseRenderer::renderCourse(mRenderer, mCamera, mSlalomCoordinator);
    }

    if (mMooseCoordinator.isEnabled()) {
        MooseTestRenderer::renderCourse(mRenderer, mCamera, mMooseCoordinator);
    }

    if (mSkidpadCoordinator.isEnabled()) {
        SkidpadRenderer::renderCourse(mRenderer, mCamera, mSkidpadCoordinator);
    }

    mWindRenderer.renderVectorField(mRenderer, mWindCoordinator, mCamera, &mBody);

    const auto v = mBody.getLinearVelocity();
    const double speedMps { v.length() };
    const double speedMph { speedMps / 0.44704 };

    TelemetrySnapshot snapshot { };
    snapshot.speedMps = speedMps;
    snapshot.speedMph = speedMph;
    snapshot.angularVelocity = mBody.getAngularVelocity();
    snapshot.stepDurationMs = mConfig.fixedDt * 1000.0;
    snapshot.droppedSeconds = mDroppedTime;
    snapshot.fps = mCurrentFps;
    snapshot.barrierOn = mBarrier;
    snapshot.gravityOn = mGravity;
    snapshot.deadZoneOn = mCamera.isDeadZoneEnabled();
    snapshot.speedZoomOn = mCamera.isSpeedZoomEnabled();
    snapshot.inputHudVisible = mInputHudVisible;
    snapshot.allHudVisible = mAllHudVisible;
    snapshot.steerLeft = mKeyA;
    snapshot.steerRight = mKeyD;
    snapshot.throttleForward = mKeyW;
    snapshot.throttleReverse = mKeyS;
    snapshot.handbrake = mKeySpace;
    snapshot.isPaused = mPaused;
    snapshot.terrainName = std::string(vehicle_sim::terrain::SurfaceMaterial::getName(getTerrain()));
    snapshot.terrainFriction = mCar.getSurfaceMaterial().friction;
    snapshot.terrainRollingDrag = mCar.getSurfaceMaterial().rollingDrag;
    snapshot.vehicleName = std::string(getVehicleName());

    snapshot.circuitMode = mCircuitCoordinator.isEnabled();
    snapshot.slalomMode = mSlalomCoordinator.isEnabled();
    snapshot.mooseMode = mMooseCoordinator.isEnabled();
    snapshot.skidpadMode = mSkidpadCoordinator.isEnabled();
    if (mCircuitCoordinator.isEnabled()) {
        const auto& metrics = mCircuitCoordinator.getMetrics();
        snapshot.currentLapTime = metrics.currentLapTime;
        snapshot.bestLapTime = metrics.bestLapTime;
        snapshot.lastLapTime = metrics.lastLapTime;
        snapshot.currentSector = metrics.currentSector;
        snapshot.completedLaps = metrics.completedLaps;
        snapshot.terrainName
            = std::string(vehicle_sim::terrain::SurfaceMaterial::getName(mCircuitCoordinator.getActiveMaterial().type));
        snapshot.terrainFriction = mCircuitCoordinator.getActiveMaterial().friction;
        snapshot.terrainRollingDrag = mCircuitCoordinator.getActiveMaterial().rollingDrag;
    }

    switch (mVehicleType) {
    case MotionLabVehicleType::ClassicCar:
        snapshot.vehicleDetails = mCar.isSkidding() ? "TIRES SKIDDING" : "GRIP OK";
        break;
    case MotionLabVehicleType::RaceCar:
        snapshot.vehicleDetails = mRaceCar.getDrsAngleDeg() > 1.0 ? "DRS ACTIVE (LOW DRAG)" : "AERO DOWNFORCE ENGAGED";
        break;
    case MotionLabVehicleType::Rocket:
        snapshot.vehicleDetails = std::string("FUEL ")
            + std::to_string(static_cast<int>(mRocket.getFuelRatio() * 100.0)) + "% / THRUST "
            + std::to_string(static_cast<int>(mRocket.getThrustForce() / 1000.0)) + " KN";
        break;
    case MotionLabVehicleType::Helicopter:
        snapshot.vehicleDetails = std::string("ROTOR ") + std::to_string(static_cast<int>(mHelicopter.getRotorSpeed()))
            + " RPM / " + (mHelicopter.isAirborne() ? "AIRBORNE" : "SURFACE");
        break;
    case MotionLabVehicleType::Hovercraft:
        snapshot.vehicleDetails = mHovercraft.isHovering() ? "AIR CUSHION ACTIVE" : "SKIRT BRAKING";
        break;
    case MotionLabVehicleType::TrackedTank:
        snapshot.vehicleDetails = mTank.getTelemetry().isRecoiling ? "CANNON FIRING RECOIL" : "DUAL TRACK READY";
        break;
    case MotionLabVehicleType::AntiGravity:
        snapshot.vehicleDetails = std::string("ALT: ")
            + std::to_string(mAntiGravity.getTelemetry().rideHeight).substr(0, 4)
            + "m | BOOST: " + std::to_string(static_cast<int>(mAntiGravity.getTelemetry().boostEnergy / 3.5 * 100.0))
            + "% | " + (mAntiGravity.getTelemetry().isDrifting ? "MAGLEV DRIFT" : "MAG-LOCKED");
        break;
    case MotionLabVehicleType::DriftCar:
        snapshot.vehicleDetails = std::string("DRIFT: ")
            + std::to_string(static_cast<int>(std::abs(mDriftCar.getDriftAngle() * 57.2958))) + " DEG | "
            + (mDriftCar.isDrifting() ? "DRIFT SUSTAINED" : "GRIP OK");
        break;
    case MotionLabVehicleType::MonsterTruck:
        snapshot.vehicleDetails
            = std::string("GEAR: ") + std::to_string(mMonsterCar.getGear()) + " | 4WD MONSTER TRUCK ACTIVE";
        break;
    case MotionLabVehicleType::SemiTruck:
        snapshot.vehicleDetails = mArticulatedTruck.getTelemetry().isJackknifed ? "JACKKNIFE WARNING! TROLLEY BRAKE"
                                                                                : std::string("HITCH: ")
                + std::to_string(static_cast<int>(mArticulatedTruck.getTelemetry().hitchAngle * 57.2958)) + " DEG";
        break;
    case MotionLabVehicleType::Forklift:
        snapshot.vehicleDetails = mForklift.isTippedOver()
            ? "TIP-OVER DETECTED!"
            : (std::string("ROLLOVER: ") + std::to_string(static_cast<int>(mForklift.getRolloverIndex() * 100.0))
                  + "% | REAR STEER");
        break;
    case MotionLabVehicleType::QuadcopterDrone:
        snapshot.vehicleDetails = std::string("THRUST: ") + std::to_string(static_cast<int>(mDrone.getTotalThrust()))
            + " N | 4-ROTOR UAV FLIGHT";
        break;
    }

    ExtendedTelemetrySnapshot extendedSnapshot { };
    mTelemetryCoordinator.populateSnapshot(extendedSnapshot);
    extendedSnapshot.advancedHudVisible = mAdvancedHudVisible && mAllHudVisible;

    TelemetryHud::render(mRenderer, snapshot, extendedSnapshot, mConfig.windowWidth, mConfig.windowHeight);

    if (mAllHudVisible) {
        if (mCircuitCoordinator.isEnabled()) {
            CircuitTrackRenderer::renderRadar(
                mRenderer, mCircuitCoordinator, mBody.getPosition(), mConfig.windowWidth - 170, 30, 140, 140);
        }

        if (mSlalomCoordinator.isEnabled()) {
            SlalomCourseRenderer::renderHud(mRenderer, mSlalomCoordinator, 10, mConfig.windowHeight - 145);
        }

        if (mMooseCoordinator.isEnabled()) {
            MooseTestRenderer::renderHud(mRenderer, mMooseCoordinator, 10, mConfig.windowHeight - 155);
        }

        if (mSkidpadCoordinator.isEnabled()) {
            SkidpadRenderer::renderHud(mRenderer, mSkidpadCoordinator, 10, mConfig.windowHeight - 155);
        }

        if (mWindHudVisible) {
            const SDL_Point cardPos = WindVectorFieldRenderer::computeHudCardPos(
                mConfig.windowWidth,
                mConfig.windowHeight,
                mCircuitCoordinator.isEnabled(),
                mAdvancedHudVisible);

            WindVectorFieldRenderer::renderHudCard(
                mRenderer, mWindCoordinator.getSnapshot(), cardPos.x, cardPos.y);
        }

        if (mTireThermalHudVisible) {
            const bool pgActive = mSlalomCoordinator.isEnabled() || mMooseCoordinator.isEnabled()
                || mSkidpadCoordinator.isEnabled();
            const SDL_Point cardPos = TireThermalRenderer::computeHudCardPos(
                mConfig.windowWidth,
                mConfig.windowHeight,
                mCircuitCoordinator.isEnabled(),
                mAdvancedHudVisible,
                mWindHudVisible,
                pgActive);

            mTireThermalRenderer.renderHudCard(mRenderer, mTireThermalCoordinator, cardPos.x, cardPos.y);
        }
    }

    SDL_RenderPresent(mRenderer);
}

void MotionLabApp::run()
{
    constexpr int kMaxSubSteps { 12 };
    const double fixedDt { mConfig.fixedDt };

    uint64_t lastCounter { SDL_GetPerformanceCounter() };
    const double counterFreq { static_cast<double>(SDL_GetPerformanceFrequency()) };

    while (mRunning) {
        processEvents();

        const uint64_t currentCounter { SDL_GetPerformanceCounter() };
        double elapsed { static_cast<double>(currentCounter - lastCounter) / counterFreq };
        lastCounter = currentCounter;

        if (elapsed > 0.25) {
            elapsed = 0.25;
        }

        // FPS tracking
        ++mFrameCounter;
        mFpsTimer += elapsed;
        if (mFpsTimer >= 0.5) {
            mCurrentFps = static_cast<double>(mFrameCounter) / mFpsTimer;
            mFrameCounter = 0;
            mFpsTimer = 0.0;
        }

        if (!mPaused) {
            const double accepted { std::min(elapsed, fixedDt * kMaxSubSteps) };
            mDroppedTime += (elapsed - accepted);
            mTimeAccumulator += accepted;

            int steps { 0 };
            while (mTimeAccumulator >= fixedDt && steps < kMaxSubSteps) {
                stepPhysics(fixedDt);
                mTimeAccumulator -= fixedDt;
                ++steps;
            }
        }

        mCamera.update(mBody.getPosition(), mBody.getLinearVelocity(), elapsed);
        mWindRenderer.updateParticles(mWindCoordinator, mCamera, elapsed, &mBody);
        renderFrame();
    }
}

bool MotionLabApp::isRunning() const noexcept
{
    return mRunning;
}

bool MotionLabApp::isPaused() const noexcept
{
    return mPaused;
}

bool MotionLabApp::isBarrierActive() const noexcept
{
    return mBarrier;
}

bool MotionLabApp::isGravityActive() const noexcept
{
    return mGravity;
}

const vehicle_sim::dynamics::MotionLabCarDynamics& MotionLabApp::getCar() const noexcept
{
    return mCar;
}

vehicle_sim::dynamics::MotionLabCarDynamics& MotionLabApp::getCar() noexcept
{
    return mCar;
}

const vehicle_sim::dynamics::RaceCarDynamics& MotionLabApp::getRaceCar() const noexcept
{
    return mRaceCar;
}

vehicle_sim::dynamics::RaceCarDynamics& MotionLabApp::getRaceCar() noexcept
{
    return mRaceCar;
}

const vehicle_sim::dynamics::RocketDynamics& MotionLabApp::getRocket() const noexcept
{
    return mRocket;
}

vehicle_sim::dynamics::RocketDynamics& MotionLabApp::getRocket() noexcept
{
    return mRocket;
}

const vehicle_sim::dynamics::HelicopterDynamics& MotionLabApp::getHelicopter() const noexcept
{
    return mHelicopter;
}

vehicle_sim::dynamics::HelicopterDynamics& MotionLabApp::getHelicopter() noexcept
{
    return mHelicopter;
}

const vehicle_sim::dynamics::HovercraftDynamics& MotionLabApp::getHovercraft() const noexcept
{
    return mHovercraft;
}

vehicle_sim::dynamics::HovercraftDynamics& MotionLabApp::getHovercraft() noexcept
{
    return mHovercraft;
}

const vehicle_sim::dynamics::TrackedTankDynamics& MotionLabApp::getTank() const noexcept
{
    return mTank;
}

vehicle_sim::dynamics::TrackedTankDynamics& MotionLabApp::getTank() noexcept
{
    return mTank;
}

const vehicle_sim::dynamics::AntiGravityRacerDynamics& MotionLabApp::getAntiGravity() const noexcept
{
    return mAntiGravity;
}

vehicle_sim::dynamics::AntiGravityRacerDynamics& MotionLabApp::getAntiGravity() noexcept
{
    return mAntiGravity;
}

const vehicle_sim::dynamics::DriftCarDynamics& MotionLabApp::getDriftCar() const noexcept
{
    return mDriftCar;
}

vehicle_sim::dynamics::DriftCarDynamics& MotionLabApp::getDriftCar() noexcept
{
    return mDriftCar;
}

const vehicle_sim::dynamics::MonsterCarDynamics& MotionLabApp::getMonsterCar() const noexcept
{
    return mMonsterCar;
}

vehicle_sim::dynamics::MonsterCarDynamics& MotionLabApp::getMonsterCar() noexcept
{
    return mMonsterCar;
}

const vehicle_sim::dynamics::ArticulatedTruckDynamics& MotionLabApp::getArticulatedTruck() const noexcept
{
    return mArticulatedTruck;
}

vehicle_sim::dynamics::ArticulatedTruckDynamics& MotionLabApp::getArticulatedTruck() noexcept
{
    return mArticulatedTruck;
}

const vehicle_sim::dynamics::ForkliftDynamics& MotionLabApp::getForklift() const noexcept
{
    return mForklift;
}

vehicle_sim::dynamics::ForkliftDynamics& MotionLabApp::getForklift() noexcept
{
    return mForklift;
}

const vehicle_sim::dynamics::QuadcopterDroneDynamics& MotionLabApp::getDrone() const noexcept
{
    return mDrone;
}

vehicle_sim::dynamics::QuadcopterDroneDynamics& MotionLabApp::getDrone() noexcept
{
    return mDrone;
}

const vehicle_sim::physics::RigidBody2D& MotionLabApp::getBody() const noexcept
{
    return mBody;
}

vehicle_sim::physics::RigidBody2D& MotionLabApp::getBody() noexcept
{
    return mBody;
}

const MotionLabCamera& MotionLabApp::getCamera() const noexcept
{
    return mCamera;
}

MotionLabCamera& MotionLabApp::getCamera() noexcept
{
    return mCamera;
}

const MotionLabAppConfig& MotionLabApp::getConfig() const noexcept
{
    return mConfig;
}

bool MotionLabApp::toggleAdvancedHud() noexcept
{
    mAdvancedHudVisible = !mAdvancedHudVisible;
    return mAdvancedHudVisible;
}

bool MotionLabApp::isAdvancedHudVisible() const noexcept
{
    return mAdvancedHudVisible;
}

void MotionLabApp::setAdvancedHudVisible(bool visible) noexcept
{
    mAdvancedHudVisible = visible;
}

bool MotionLabApp::toggleAllHud() noexcept
{
    mAllHudVisible = !mAllHudVisible;
    return mAllHudVisible;
}

bool MotionLabApp::isAllHudVisible() const noexcept
{
    return mAllHudVisible;
}

void MotionLabApp::setAllHudVisible(bool visible) noexcept
{
    mAllHudVisible = visible;
}

bool MotionLabApp::toggleRecording() noexcept
{
    return mTelemetryCoordinator.toggleRecording();
}

const TelemetryCoordinator& MotionLabApp::getTelemetryCoordinator() const noexcept
{
    return mTelemetryCoordinator;
}

TelemetryCoordinator& MotionLabApp::getTelemetryCoordinator() noexcept
{
    return mTelemetryCoordinator;
}

const WindTurbulenceCoordinator& MotionLabApp::getWindCoordinator() const noexcept
{
    return mWindCoordinator;
}

WindTurbulenceCoordinator& MotionLabApp::getWindCoordinator() noexcept
{
    return mWindCoordinator;
}

const WindVectorFieldRenderer& MotionLabApp::getWindRenderer() const noexcept
{
    return mWindRenderer;
}

WindVectorFieldRenderer& MotionLabApp::getWindRenderer() noexcept
{
    return mWindRenderer;
}

bool MotionLabApp::toggleWindHud() noexcept
{
    mWindHudVisible = !mWindHudVisible;
    return mWindHudVisible;
}

bool MotionLabApp::isWindHudVisible() const noexcept
{
    return mWindHudVisible;
}

void MotionLabApp::setWindHudVisible(bool visible) noexcept
{
    mWindHudVisible = visible;
}

void MotionLabApp::updateTireThermals(double dt) noexcept
{
    if (!mTireThermalCoordinator.isWheeledVehicle()) {
        return;
    }

    const double airSpeed { mBody.getLinearVelocity().length() };
    const double forwardSpeed { mBody.toLocalVec(mBody.getLinearVelocity()).x };

    if (mVehicleType == MotionLabVehicleType::DriftCar) {
        const auto& wheels = mDriftCar.getWheelStates();
        for (std::size_t i { 0U }; i < 4U; ++i) {
            const auto& w = wheels[i];
            const auto worldPos = mBody.toWorldPoint(w.localOffset);
            constexpr double wheelRadius { 0.32 };
            const double safeVx { std::max(1.0, std::abs(forwardSpeed)) };
            const double omega { (forwardSpeed + w.slipRatio * safeVx) / wheelRadius };
            const double vsx { w.slipRatio * safeVx };
            const double vsy { -std::tan(w.slipAngle) * safeVx };
            mTireThermalCoordinator.updateWheel(i, w.tractiveForce, w.corneringForce, w.normalLoad,
                                                vsx, vsy, omega, wheelRadius, airSpeed, worldPos, dt, mTireSmokeBuffer);
        }
    } else if (mVehicleType == MotionLabVehicleType::RaceCar) {
        const auto& wheels = mRaceCar.getWheelStates();
        for (std::size_t i { 0U }; i < 4U; ++i) {
            const auto& w = wheels[i];
            const auto worldPos = mBody.toWorldPoint(w.localOffset);
            constexpr double wheelRadius { 0.33 };
            const double safeVx { std::max(1.0, std::abs(forwardSpeed)) };
            const double omega { (forwardSpeed + w.slipRatio * safeVx) / wheelRadius };
            const double vsx { w.slipRatio * safeVx };
            const double vsy { -std::tan(w.slipAngle) * safeVx };
            mTireThermalCoordinator.updateWheel(i, w.tractiveForce, w.corneringForce, w.normalLoad,
                                                vsx, vsy, omega, wheelRadius, airSpeed, worldPos, dt, mTireSmokeBuffer);
        }
    } else {
        const double totalMass { mBody.getMass() };
        constexpr double gravityAcc { 9.81 };
        const double normalLoadPerWheel { (totalMass * gravityAcc) * 0.25 };
        const double lateralSpeed { mBody.toLocalVec(mBody.getLinearVelocity()).y };
        const double safeForwardSpeed { std::max(1.0, std::abs(forwardSpeed)) };
        const double slipAngle { -std::atan2(lateralSpeed, safeForwardSpeed) };

        const vehicle_sim::math::Vec2 offsets[4] = {
            { 1.3, 0.8 }, { 1.3, -0.8 }, { -1.3, 0.8 }, { -1.3, -0.8 }
        };

        const double tractiveDemand { (mKeyW ? 2500.0 : 0.0) - (mKeyS ? 4000.0 : 0.0) };
        for (std::size_t i { 0U }; i < 4U; ++i) {
            const auto worldPos = mBody.toWorldPoint(offsets[i]);
            constexpr double wheelRadius { 0.32 };
            const double omega { forwardSpeed / wheelRadius };
            const double vsx { (i >= 2U && mKeyW) ? 1.5 : 0.0 };
            const double vsy { lateralSpeed };
            const double corneringForce { normalLoadPerWheel * std::sin(slipAngle) };
            mTireThermalCoordinator.updateWheel(i, tractiveDemand * 0.25, corneringForce, normalLoadPerWheel,
                                                vsx, vsy, omega, wheelRadius, airSpeed, worldPos, dt, mTireSmokeBuffer);
        }
    }

    mTireSmokeBuffer.update(dt);
}

const TireThermalCoordinator& MotionLabApp::getTireThermalCoordinator() const noexcept
{
    return mTireThermalCoordinator;
}

TireThermalCoordinator& MotionLabApp::getTireThermalCoordinator() noexcept
{
    return mTireThermalCoordinator;
}

const TireThermalRenderer& MotionLabApp::getTireThermalRenderer() const noexcept
{
    return mTireThermalRenderer;
}

TireThermalRenderer& MotionLabApp::getTireThermalRenderer() noexcept
{
    return mTireThermalRenderer;
}

const TireSmokeParticleBuffer& MotionLabApp::getTireSmokeBuffer() const noexcept
{
    return mTireSmokeBuffer;
}

TireSmokeParticleBuffer& MotionLabApp::getTireSmokeBuffer() noexcept
{
    return mTireSmokeBuffer;
}

void MotionLabApp::toggleTireThermalHud() noexcept
{
    mTireThermalHudVisible = !mTireThermalHudVisible;
    mTireThermalRenderer.setHudVisible(mTireThermalHudVisible);
}

void MotionLabApp::resetTireThermalStates() noexcept
{
    mTireThermalCoordinator.reset();
    mTireSmokeBuffer.clear();
}

} // namespace motion_lab
