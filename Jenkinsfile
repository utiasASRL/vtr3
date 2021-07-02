pipeline {
    agent none
    stages {
        stage("build dependencies") {
            agent { dockerfile true }
            environment {
                HOME = '.'
            }
            stages {
                stage('build') {
                    steps {
                        sh '''
                            export VTRSRC=`pwd`
                            source ${VTRVENV}/bin/activate
                            source ${VTRDEPS}/vtr_ros2_deps/install/setup.bash
                            cd ${VTRSRC}/main
                            touch ${VTRSRC}/main/src/vtr_interface/COLCON_IGNORE  # TODO having trouble running on server
                            colcon build --symlink-install
                        '''
                    }
                }
                stage('unit tests') {
                    steps {
                        sh '''
                            echo "TODO"
                        '''
                    }
                }
            }
            post {
                always {
                    deleteDir()
                }
            }
        }
    }
}