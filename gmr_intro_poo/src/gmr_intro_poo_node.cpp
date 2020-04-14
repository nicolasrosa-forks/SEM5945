/* =========== */
/*  Libraries  */
/* =========== */
#include "../include/gmr_intro_poo/gmr_intro_poo.hpp"

/* =========== */
/*  Functions  */
/* =========== */

/* ====== */
/*  Main  */
/* ====== */
int main(int argc, char **argv){
    // Inicialização da ROS no contexto deste nó
    ros::init(argc, argv, "gmr_intro_poo_node");
    ros::NodeHandle nh("~");

    // Criação de uma instância da classe RobotClass em que
    // um ponteiro a ros::NodeHandle é passado como argumento
    RobotClass robot(&nh);

    // Loop de ROS para manter o nó vivo
    ros::spin();
}



