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
                            set +x  # Disable output from source command
                            export VTRSRC=`pwd`
                            source ${VTRDEPS}/vtr_ros2_deps/install/setup.bash
                            set -x
                            cd ${VTRSRC}/main
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