#!/bin/bash
# System - Master Launch Script
################################################################################

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

ROBOT_NAME="jetauto_1"
SLAM_METHOD="gmapping"
LOG_DIR="$HOME/.jetauto_delivery/logs"

mkdir -p "$LOG_DIR"

### STEP 0 — Environment setup
echo "Sourcing ROS Melodic environment..."
source /opt/ros/melodic/setup.bash
source ~/Desktop/seruCP2/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_HOSTNAME=$ROS_HOSTNAME"

cleanup() {
    echo ""
    echo -e "${RED}⚠️  Caught Ctrl+C — stopping all ROS processes...${NC}"
    pkill -f roscore
    pkill -f roslaunch
    pkill -f rosmaster
    pkill -f rviz
    pkill -f delivery_system
    pkill -f object_detection
    echo -e "${GREEN}✅ All background ROS nodes terminated.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM

################################################################################
# UTILITY FUNCTIONS
################################################################################

print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}→ $1${NC}"
}

################################################################################
# KILL LEFTOVER ROSCORE
################################################################################

kill_leftover_roscore() {
    print_info "Checking for existing roscore..."
    if pgrep -x "roscore" > /dev/null; then
        print_info "Old roscore detected. Killing..."
        pkill -9 roscore
        pkill -9 rosmaster
        sleep 2
        print_success "Old roscore killed"
    else
        print_success "No previous roscore running"
    fi
}

################################################################################
# START ROSCORE
################################################################################

start_roscore() {
    print_info "Starting roscore..."
    roscore > /tmp/roscore.log 2>&1 &
    sleep 5
    if ! pgrep -x "roscore" > /dev/null; then
        print_error "Failed to start roscore. Check /tmp/roscore.log"
        exit 1
    fi
    print_success "roscore started successfully"
}

################################################################################
# MAPPING PHASE - JOYSTICK CONTROLLED (GMAPPING)
################################################################################

gmap_launch(){
    print_header "Mapping Phase - Joystick Control (Gmapping)"
    
    kill_leftover_roscore 
    start_roscore
    sleep 2
    
    print_info "Stopping JetAuto app service..."
    sudo systemctl stop start_app_node.service
    sleep 2
    print_success "App service stopped"
    
    print_info "Launching SLAM service..."
    roslaunch jetauto_slam slam.launch slam_methods:=gmapping &
    sleep 2
    
    print_info "Launching RViz visualization..."
    roslaunch jetauto_slam rviz_slam.launch slam_methods:=gmapping &
    sleep 2
    
    print_info "Launching joystick controller..."
    roslaunch jetauto_peripherals joystick_control.launch &
    sleep 2
    
    print_success "Mapping phase started"
    print_info "Use joystick to drive robot through delivery area"
    print_info "RViz is running and displaying the map in real-time"
    print_info ""
    
    # Keep script alive until Ctrl+C is pressed
    while true; do
        sleep 1
    done

}

################################################################################
# SAVE MAP
################################################################################

save_map(){
    print_header "Saving Map"
    
    print_info "Saving map to ~/Desktop/seruCP2/src/jetauto_slam/maps/my_map..."
    rosrun map_server map_saver -f ~/Desktop/seruCP2/src/jetauto_slam/maps/CRIA_LAB map:=/jetauto_1/map
    
    print_success "Map saved successfully"
    print_info ""
}

################################################################################
# NAVIGATION PHASE (GMAPPING)
################################################################################

gmap_navi(){
    print_header "Navigation Phase - Autonomous Delivery (Gmapping)"
    kill_leftover_roscore
    start_roscore
    sleep 2

    print_info "Stopping JetAuto app service..."
    sudo systemctl stop start_app_node.service
    sleep 4
    print_success "App service stopped"

    #print_info "Launch 1"
    #roslaunch jetauto_navigation navigation.launch map:=CRIA_LAB robot_name:=jetauto_1 &

    print_info "Launching navigation script..."
    bash ~/Desktop/seruCP2/src/jetauto_bringup/scripts/navigation.sh &

    sleep 5

    print_success "Navigation phase started"
    print_info "Set pose: Use '2D Pose Estimate' to initialize robot position"
    print_info "Set goal: Use '2D Nav Goal' to set delivery destination"
    print_info ""
}

################################################################################
# DELIVERY PHASE
################################################################################

delivery_phase(){
    print_header "Delivery Phase - Autonomous Delivery System"
    
    print_info "Launching delivery system..."
    sleep 2
    
    # Launch delivery system as a ROS node
    python3 ~/Desktop/seruCP2/src/jetauto_bringup/scripts/delivery.py
}

################################################################################
# MENU & MAIN
################################################################################

show_menu() {
    print_header "JetAuto Delivery System"
    echo ""
    echo "Commands:"
   
    echo "  gmap                 Start mapping (Gmapping - joystick controlled)"
    echo "  save                 Save the current map"
    echo "  gnavi                Start navigation (Gmapping - autonomous)"
    echo "  delivery             Start delivery system (autonomous delivery with user selection)"
    echo "  full                 Start navigation then delivery"
    echo "  help                 Show this menu"
    echo "" 
}

main() {

    [ $# -eq 0 ] && { show_menu; exit 0; }
    
    case "$1" in
        gmap) gmap_launch ;;              
        save) save_map ;;                 
        gnavi) gmap_navi ;;               
        delivery) delivery_phase ;;        
        full) gmap_navi; sleep 20; delivery_phase ;;
        help) show_menu ;;                
        *) print_error "Unknown command: $1"; show_menu; exit 1 ;;
    esac
}

main "$@"
