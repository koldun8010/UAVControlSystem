#include "server.h"
#include <iostream>
#include <plugins/action/action.h>
#include <plugins/telemetry/telemetry.h>
#include <thread>
#include <chrono>

Server::Server(std::string uav_url) {
    setUrl(uav_url);
    connect();
}

void Server::setUrl(std::string uav_url) {
    this->connection_url = uav_url;
}

void Server::connect() {
    this->connection_result = this->mavsdk.add_any_connection(this->connection_url);
    if (this->connection_result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Ошибка подключения\n";
    } else {
        std::cout << "Подключение установлено\n";
    }

    /*
    while (this->mavsdk.systems().size() == 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));                                     //ЗАГОТОВКА ПОД ПОИСК СИСТЕМЫ
    }


    this->system = mavsdk.systems().at(0);
    */
}

void Server::takeOff() {
    auto action = mavsdk::Action{this->system};

    const mavsdk::Action::Result arm_result = action.arm();
    if (arm_result != mavsdk::Action::Result::Success) {
        std::cerr << "Не удалось вооружить: " << arm_result << std::endl;

        return;
    }

    if (action.get_takeoff_altitude().second == 0.0) {
        action.set_takeoff_altitude(15.0);
    }

    const mavsdk::Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != mavsdk::Action::Result::Success) {
        std::cerr << "Не удалось выполнить взлёт: " << takeoff_result << std::endl;
    } else {
        std::cout << "Взлет совершен\n";
    }
}

void Server::land() {
    auto action = mavsdk::Action{this->system};
    const mavsdk::Action::Result land_result = action.land();

    if (land_result != mavsdk::Action::Result::Success) {
        std::cerr << "Не удалось выполнить приземление: " << land_result << std::endl;

            return;
    }

    const mavsdk::Action::Result disarm_result = action.disarm();
    if (disarm_result != mavsdk::Action::Result::Success) {
        std::cerr << "Не удалось выполнить деактивацию: " << disarm_result << std::endl;
    }
}

void Server::returnToLaunch() {
    auto action = mavsdk::Action(this->system);
    const mavsdk::Action::Result rtl_result = action.return_to_launch();

    if (rtl_result != mavsdk::Action::Result::Success) {
        std::cerr << "Не удалось вернуться на точу отправки" << rtl_result << std::endl;
    }
}

double Server::getLatitude() {

    return latitude;
}

double Server::getLongitude() {

    return longitude;
}

float Server::getAltitude() {
    return altitude;
}

float Server::getRollDegree() {

    return rollDegree;
}

float Server::getPitchDegree() {

    return pitchDegree;
}

float Server::getYawDegree() {

    return yawDegree;
}

float Server::getVelocity() {
    return velocity;
}

float Server::getVelocityN() {
    return velocityN;
}

float Server::getVelocityE() {
    return velocityE;
}

float Server::getVelocityD() {
    return velocityD;
}

bool Server::checkCoordinates(double lat, double lon) {
    auto telemetry = mavsdk::Telemetry{this->system};
    auto position = telemetry.position();
    auto velocity_ned = telemetry.velocity_ned();

    double absVel = std::sqrt(velocity_ned.north_m_s * velocity_ned.north_m_s +
                                      velocity_ned.east_m_s * velocity_ned.east_m_s +
                                      velocity_ned.down_m_s * velocity_ned.down_m_s);

    this->updateInformation(position.latitude_deg,
                            position.longitude_deg,
                            position.absolute_altitude_m,
                            telemetry.attitude_euler().roll_deg,
                            telemetry.attitude_euler().pitch_deg,
                            telemetry.attitude_euler().yaw_deg,
                            velocity_ned.north_m_s,
                            velocity_ned.east_m_s,
                            velocity_ned.down_m_s,
                            absVel
                            );
    if (latitude != lat || longitude != lon) {

        return false;
    }

    return true;
}

void Server::updateInformation(double lat, double lon, float alt, float rollDeg, float pitchDeg, float yawDeg, float vel, float velN, float velE, float velD) {
    setLatitude(lat);
    setLongitude(lon);
    setAltitude(alt);
    setRollDegree(rollDeg);
    setPitchDegree(pitchDeg);
    setYawDegree(yawDeg);
    setVelocity(vel);
    setVelocityN(velN);
    setVelocityE(velE);
    setVelocityD(velD);
}

void Server::setLatitude(double lat) {
    latitude = lat;
}

void Server::setLongitude(double lon) {
    longitude = lon;
}

void Server::setAltitude(float alt) {
    altitude = alt;
}

void Server::setRollDegree(float rollDeg) {
    rollDegree = rollDeg;
}

void Server::setPitchDegree(float pitchDeg) {
    pitchDegree = pitchDeg;
}

void Server::setYawDegree(float yawDeg) {
    yawDegree = yawDeg;
}

void Server::setVelocity(float velocity) {
    this->velocity = velocity;
}

void Server::setVelocityN(float velocityN) {
    this->velocityN = velocityN;
}

void Server::setVelocityE(float velocityE) {
    this->velocityE = velocityE;
}

void Server::setVelocityD(float velocityD) {
    this->velocityD = velocityD;
}

void Server::goTo(double lat, double lon, double absAlt, double yawDeg) {
    auto action = mavsdk::Action{this->system};

    action.goto_location(lat, lon, absAlt, yawDeg);
}

void Server::setWaypoint(double lat, double longt, double relativeAlt, double speed) {
    std::shared_ptr<mavsdk::Mission::MissionItem> newItem(new mavsdk::Mission::MissionItem());
    newItem->longitude_deg = lat;
    newItem->latitude_deg = longt;
    newItem->relative_altitude_m = relativeAlt;
    newItem->speed_m_s = speed;

    this->missionItems.push_back(newItem);
    this->mission_plan = mavsdk::Mission::MissionPlan{};
}

void Server::startMission() {
    auto mission = mavsdk::Mission{this->system};
    //this->mission_plan = mavsdk::Mission::MissionPlan{};
    //this->mission_plan.mission_items = missionItems;                                      //0 ИНФЫ ЧТО НЕ ТАК

    const mavsdk::Mission::Result result = mission.upload_mission(
        this->mission_plan);

    if (result != mavsdk::Mission::Result::Success) {
        std::cout << "Mission upload failed (" << result << "), exiting." << '\n';

        return;
    }

    std::cout << "Mission uploaded." << '\n';
    mission.start_mission();
}
