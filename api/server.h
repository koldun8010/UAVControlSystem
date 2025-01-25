#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>

class Server {
    private:
        mavsdk::Mavsdk mavsdk;
        std::string connection_url;
        mavsdk::ConnectionResult connection_result;
        std::shared_ptr<mavsdk::System> system;
        std::vector<std::shared_ptr<mavsdk::Mission::MissionItem>> missionItems;
        mavsdk::Mission::MissionPlan mission_plan;
        double latitude;
        double longitude;
        float altitude;
        float rollDegree;
        float pitchDegree;
        float yawDegree;
        float velocity;
        float velocityN;
        float velocityE;
        float velocityD;

    public:
        explicit Server(std::string uav_url);

        void setUrl(std::string uav_url);

        void connect();

        void takeOff();

        void land();

        void returnToLaunch();

        double getLatitude();

        double getLongitude();

        float getAltitude();

        float getRollDegree();

        float getPitchDegree();

        float getYawDegree();

        float getVelocity();

        float getVelocityN();

        float getVelocityE();

        float getVelocityD();

        bool checkCoordinates(double lat, double lon);

        void updateInformation(double lat, double lon, float alt, float rollDeg, float pitchDeg, float yawDeg, float vel, float velN, float velE, float velD);

        void setAltitude(float alt);

        void setLatitude(double lat);

        void setLongitude(double lon);

        void setRollDegree(float rollDeg);

        void setPitchDegree(float pitchDeg);

        void setYawDegree(float yawDeg);

        void setVelocity(float vel);

        void setVelocityN(float velN);

        void setVelocityE(float velE);

        void setVelocityD(float velD);

        void goTo(double lat, double lon, double alt, double yawDeg);

        void setWaypoint(double lat, double lon, double alt, double speed);

        void startMission();
};
