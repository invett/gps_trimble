#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <sstream>

#include <gps_common/conversions.h>

#define MSG_SZ 100
#define PCAP_TCP_SLL_HEADER 68
#define PCAP_TCP_HEADER 66

class GpsPublisher
{
public:

    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Publisher gga_pub_;
    ros::Publisher navsat_pub_;
    ros::Publisher utm_pub_;
    int port;
    std::string ip;


  GpsPublisher() : nh_("~")
  {

    // Obtener los valores de port e ip de los parámetros del nodo
    nh_.param<int>("port", port, 5018); //Establece valor por defecto 5018
    nh_.param<std::string>("ip", ip, "192.168.254.12"); //Establece valor por defecto 192.168.254.12
    // Crear el publicador para el mensaje de tipo String
    gga_pub_ = nh_.advertise<std_msgs::String>("trimble/gga", 10);

    // Crear el publicador para el mensaje de tipo NavSatFix
    navsat_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("trimble/navsat_fix", 10);
    utm_pub_ = nh_.advertise<geometry_msgs::Point>("trimble/position", 10);

    // Crear un temporizador para ejecutar el método de callback cada 0.5 segundos
    timer_ = nh_.createTimer(ros::Duration(0.5), &GpsPublisher::timerCallback, this,false);
  }

private:
  void timerCallback(const ros::TimerEvent& event)
  {
    std_msgs::String gga_message;  // Mensaje de tipo String para la cadena GGA
    sensor_msgs::NavSatFix navsat_message;  // Mensaje de tipo NavSatFix para los datos GPS
    geometry_msgs::Point utm;  // Mensaje con xyz para los datos GPS

    int sockFd;
    bool isGPSOpen = false;
    struct sockaddr_in serv_addr;
    char messagegps[100];
    int msgSize;

    if ((sockFd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
      ROS_ERROR("Failed to create socket");
      return;
    }

    memset(&serv_addr, '0', sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0)
    {
      ROS_ERROR("Failed to set server address");
      return;
    }

    if (connect(sockFd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
      ROS_ERROR("Failed to connect to server");
      return;
    }

    isGPSOpen = true;
    //ROS_INFO("Connected to server");

    while (ros::ok())
    {
      if ((msgSize = read(sockFd, messagegps, sizeof(messagegps) - 1)) > 0)
      {
        messagegps[msgSize] = '\0';
        //ROS_INFO("%s", messagegps);
        gga_message.data = messagegps;  // Asignar la cadena GGA al mensaje de tipo String
        gga_pub_.publish(gga_message);  // Publicar el mensaje de cadena GGA

        // Analizar el mensaje GGA y asignar los valores al mensaje de tipo NavSatFix
        std::stringstream ss(messagegps);
        std::string token;
        std::vector<std::string> tokens;

        // Dividir la cadena GGA en tokens separados por comas
        while (getline(ss, token, ','))
        {
          tokens.push_back(token);
        }

        // Verificar si el tamaño del vector es suficiente para extraer los valores
        if (tokens.size() >= 10)
        {
          // Extraer los valores necesarios de los tokens y asignarlos al mensaje NavSatFix
          try
          {
            double latitud = std::stod(tokens[2]);
            double longitud = std::stod(tokens[4]);
            double altitud = std::stod(tokens[9]);
            int calidadPos = std::stoi(tokens[6]);


            // Formato estandar de la cadena GGA
            //$GPGGA,182402.02,3436.5829,S,05825.7855,W,1,04,1.5,57,M,-34.0,M,,,*70
            // 1-Hora GPS (no se utiliza en este ejemplo).
            // 2-Latitud en formato decimal (grados y fracción de minutos).
            // 3-Indicador de latitud (N para latitud norte, S para latitud sur).
            // 4-Longitud en formato decimal (grados y fracción de minutos).
            // 5-Indicador de longitud (E para longitud este, W para longitud oeste).
            // 6-Calidad de la posición.
                // 0 Invalido
                // 1 Fijación GPS
                // 2 Ajuste DGPS
                // 3 Corrección de PPS
                // 4 RTK?? -- aniadido ruben


//            0: Fix not valid
//            1: GPS fix
//            2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
//            3: Not applicable
//            4: RTK Fixed, xFill
//            5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX
//            6: INS Dead reckoning

            // 7-Número de satélites utilizados (no se utiliza en este ejemplo).
            // 8-Precisión horizontal (no se utiliza en este ejemplo).
            // 9-Altitud sobre el nivel del mar.
            // 10-Unidad de altitud (generalmente metros).

            // Convertir el ángulo de longitud a grados decimales
            int lng_degrees = static_cast<int>(longitud / 100);
            double lng_minutes = (longitud - lng_degrees * 100.0) / 60.0;
            double lng_decimal = lng_degrees + lng_minutes;

            // Convertir el ángulo de latitud a grados decimales
            int lat_degrees = static_cast<int>(latitud / 100);
            double lat_minutes = (latitud - lat_degrees * 100.0) / 60.0;
            double lat_decimal = lat_degrees + lat_minutes;

            // Comprobar W o E
            if (tokens[5] == "W")
            {
                lng_decimal *= -1.0;
            }
            //Comprobar S o N
            if (tokens[3] == "S")
            {
                lat_decimal *= -1.0;
            }

            navsat_message.latitude = lat_decimal;
            navsat_message.longitude = lng_decimal;
            navsat_message.altitude = altitud;

            switch (calidadPos)
            {
              case 0:
                navsat_message.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
                break;
              case 1:
                navsat_message.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
                break;
              case 2:
                navsat_message.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
                break;
              case 4:
                navsat_message.status.status = 4; //RTK
              break;
              case 5:
                navsat_message.status.status = 5; // Float RTK
              break;
              default:
                navsat_message.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
                break;
            }

            navsat_message.header.stamp = ros::Time::now();
            navsat_pub_.publish(navsat_message);  // Publicar el mensaje NavSatFix

            // Northing easting
            double utm_n, utm_e, utm_z;
            std::string utm_zone;
            gps_common::LLtoUTM(lat_decimal, lng_decimal, utm_n, utm_e, utm_zone);
            //ROS_ERROR("UTM zone: %s", utm_zone.c_str());
            utm.x = utm_e;
            utm.y = utm_n;
            utm.z = altitud;
            utm_pub_.publish(utm);
          }
          catch (const std::exception& e)
          {
            ROS_ERROR("Error al convertir los valores de latitud, longitud o altitud: %s", e.what());
          }
        }
        else
        {
          ROS_WARN("La cadena GGA no contiene suficientes campos para extraer los valores GPS");
        }
      }
    }

    // Cerrar la conexión al GPS cuando finalice el nodo
    if (isGPSOpen)
    {
      close(sockFd);
      ROS_INFO("Closed connection to GPS");
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_publisher");
  GpsPublisher gpsPublisher;
  ros::spin();
  return 0;
}
