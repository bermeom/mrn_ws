#!/bin/bash

main()
{
  echo " --- [ UPC_MRN_SETUP script ] ---" 
  # This script creates a ROS workspace
  #######################Configurable#######################
  
  WSNAME="mrn_ws"
  CUSTOM_BASHRC_NAME=$WSNAME"_bashrc"
  SUDO=false
  
  #######################Logging#######################
  #REDIRECTION="/dev/null"
  DEST=${BASH_SOURCE[0]}
  FILE="${DEST##*/}"
  FILENAME="log_${FILE%.*}"
  LOGDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
  #LOGDIR="${HOME}"
  REDIRECTION="${LOGDIR}/${FILENAME}.log"
  > ${REDIRECTION}

  ROSVERSION1="kinetic"
  ROSVERSION2="indigo"
  ##OpenCV Kinetic fix
  # sudo ln -s /opt/ros/kinetic/lib/libopencv_core3.so /opt/ros/kinetic/lib/libopencv_core3.so.3.2
  
  ######################################################
  ###PARSE ARGS
  echo " --- Checking arguments"
  OPTIND=1
  while getopts ":sw:" opt; do
    [[ ${OPTARG} == -* ]] && { echo " --- Error: Missing argument for -${opt}" ; return ; }
    case $opt in
      s)
        echo " --- Enabling call to sudo commands"
        SUDO=true
        ;;
      \?)
        echo " --- Error: Invalid option: -$OPTARG"
        return
        ;;
      w)
        echo " --- Choosing workspace name: $OPTARG"
        WSNAME=$OPTARG
        CUSTOM_BASHRC_NAME=$WSNAME"_bashrc"
        ;;
      :)
        echo " --- Error: option -$OPTARG requires an argument."
        return
        ;;
    esac
  done
  ######################################################

  MYPWD=$(pwd)
  
  ######################################################
  ###CHECK ROS
  echo " --- Checking ROS presence"
  #ROSVERSION=$(ls /opt/ros | head -n 1)
  #ROSVERSION="$(rosversion -d)"
  if [ -f /opt/ros/${ROSVERSION1}/setup.bash ]; then
    ROSVERSION=${ROSVERSION1}
    echo " --- Found installed ROS ${ROSVERSION}"
  elif [ -f /opt/ros/${ROSVERSION2}/setup.bash ]; then
    ROSVERSION=${ROSVERSION2}
    echo " --- Found installed ROS ${ROSVERSION}"
    echo " --- WARNING: recommended version is ROS ${ROSVERSION1}"
  else
    echo "ERROR: no ${ROSVERSION1} or ${ROSVERSION2} ROS version installed found. Exiting"
    return
  fi

  ######################################################
  ###CUSTOM BASHRC
  echo " --- Checking custom bashrc file ~./$CUSTOM_BASHRC_NAME"

  if ! [ -f ~/.${CUSTOM_BASHRC_NAME} ];then
    echo " --- Creating custom bashrc file: ~./$CUSTOM_BASHRC_NAME to source $WSNAME workspace"
    touch ~/.${CUSTOM_BASHRC_NAME}
  fi

  if ! grep -q "source /opt/ros/${ROSVERSION}/setup.bash" ~/.${CUSTOM_BASHRC_NAME}; then
    echo "source /opt/ros/${ROSVERSION}/setup.bash" >> ~/.${CUSTOM_BASHRC_NAME}
  fi

  if ! grep -q "source ~/${WSNAME}/devel/setup.bash" ~/.${CUSTOM_BASHRC_NAME}; then
    echo "source ~/${WSNAME}/devel/setup.bash" >> ~/.${CUSTOM_BASHRC_NAME}
  fi

  ######################################################
  ###BASHRC
 
  if ! grep -q "alias $WSNAME='source ~/.${CUSTOM_BASHRC_NAME}'" ~/.bashrc; then
    echo "alias $WSNAME='source ~/.${CUSTOM_BASHRC_NAME}'" >> ~/.bashrc
    echo "$WSNAME" >> ~/.bashrc
  fi

  #################################################################
  ### WORKSPACE

  source /opt/ros/${ROSVERSION}/setup.bash
  
  if ! [ -d ~/${WSNAME}/src ];then
    echo " --- Creating ~/${WSNAME} workspace ..."
    mkdir -p ~/${WSNAME}/src
  else
    echo " --- Already found ~/$WSNAME workspace"
  fi


  if ! [ -f ~/${WSNAME}/src/CMakeLists.txt ]; then
    echo " --- Initializing ~/${WSNAME} workspace..."
    cd ~/${WSNAME}/src
    catkin_init_workspace &>> ${REDIRECTION}
  else
    echo " --- Already initialized ~/$WSNAME workspace"
  fi

  if [ ! -d ~/${WSNAME}/devel  -o  ! -d ~/${WSNAME}/build  -o  ! -f ~/${WSNAME}/devel/setup.bash ] ; then
    cd ~/${WSNAME}
    echo " --- Compiling ~/${WSNAME} workspace for first time..."
    catkin_make --force-cmake &>> ${REDIRECTION}
  else
    echo " --- Already compiled ~/$WSNAME workspace"
  fi

  source ~/${WSNAME}/devel/setup.bash
  cd ~/${WSNAME}/src

  if ! [ -f ~/${WSNAME}/src/.rosinstall ]; then
    echo " --- Initializing wstool on '${WSNAME}' workspace..."
    wstool init &>> ${REDIRECTION}
  else
    echo " --- Already initialized wstool in ~/$WSNAME workspace"
  fi

  #################################################################
  ###ADD PACKAGES
  
  add_source_pkg upc_mrn svn https://devel.iri.upc.edu/pub/labrobotica/ros/iri-ros-pkg_hydro/metapackages/iri_teaching/upc_mrn

  if ! grep -q "source /usr/share/gazebo/setup.sh" ~/.${CUSTOM_BASHRC_NAME}; then
    echo " --- Adding Gazebo paths to ~/.${CUSTOM_BASHRC_NAME}..."
    echo "source /usr/share/gazebo/setup.sh" >> ~/.${CUSTOM_BASHRC_NAME} 
  fi

  if ! grep -q "export GAZEBO_" ~/.${CUSTOM_BASHRC_NAME}; then
    echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/${WSNAME}/src/upc_mrn/models" >> ~/.${CUSTOM_BASHRC_NAME}
    #echo "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/${WSNAME}/src/upc_mrn/models" >> ~/.${CUSTOM_BASHRC_NAME}
  fi

  if ! grep -q "export TURTLEBOT_3D_SENSOR=kinect" ~/.${CUSTOM_BASHRC_NAME}; then
    echo "export TURTLEBOT_3D_SENSOR=kinect" >> ~/.${CUSTOM_BASHRC_NAME} 
  fi

  source ~/.${CUSTOM_BASHRC_NAME}

  if [ "$SUDO" = true ] ; then
    echo " --- Installing packages"
    add_apt_pkg vim
    add_apt_pkg ssh
    add_apt_pkg subversion
  
    echo " --- Installing extra ros packages"
    add_apt_rospkg navigation
    add_apt_rospkg gmapping
    add_apt_rospkg gazebo*
    add_apt_rospkg turtlebot*
    add_apt_rospkg kobuki*
    echo " --- Creating kobuki ftdi udev rule"
    rosrun kobuki_ftdi create_udev_rules &>> ${REDIRECTION}
  fi

  cd $MYPWD
  echo " --- Done."
}

function add_apt_pkg #ARGS: pkg
{
  PKG=$1
  echo " --- Installing apt-get package ${PKG}"
  sudo apt-get install -y ${PKG} &>> ${REDIRECTION} || echo "###ERROR, for more info check log file: $LOGDIR/$FILENAME.log"
}

function add_apt_rospkg #ARGS: pkg
{
  PKG=$1
  echo " --- Installing apt-get package ros-${ROSVERSION}-${PKG}"
  sudo apt-get install -y ros-${ROSVERSION}-${PKG} &>> ${REDIRECTION} || echo "###ERROR, for more info check log file: $LOGDIR/$FILENAME.log"
}

function add_source_pkg #ARGS: pkg type url
{
  PKG=$1
  TYPE=$2
  URL=$3
  
  cd ~/${WSNAME}/src
  #if ! rospack -q find ${PKG} &>> ${REDIRECTION} || rosstack -q find ${PKG} &>> ${REDIRECTION}; then
  if ! [ -d ~/${WSNAME}/src/${PKG} ]; then
    echo " --- Downloading ROS ${PKG} package from ${URL}"
    wstool set ${PKG} -y --${TYPE} ${URL} &>> ${REDIRECTION} || echo "###ERROR, for more info check log file: $LOGDIR/$FILENAME.log"
  fi
  echo " --- Updating ROS ${PKG} package from ${URL}"
  wstool update ${PKG} &>> ${REDIRECTION} || echo "###ERROR, for more info check log file: $LOGDIR/$FILENAME.log"
  source ~/${WSNAME}/devel/setup.bash
  roscd ${PKG}
  echo " --- Compiling ${PKG} package"
  cd ~/${WSNAME}
  catkin_make --only-pkg-with-deps ${PKG} &>> ${REDIRECTION} || echo "###ERROR: ${PKG} compilation failed, for more info check log file: $LOGDIR/$FILENAME.log"
  # catkin_make -DCATKIN_WHITELIST_PACKAGES="${PKG}" &>> ${REDIRECTION}
}

main "$@"