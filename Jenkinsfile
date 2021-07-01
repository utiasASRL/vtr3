pipeline {
    agent none
    stages {
        stage("build dependencies") {
            agent { dockerfile true }
            stages {
                stage('build dependence') {
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