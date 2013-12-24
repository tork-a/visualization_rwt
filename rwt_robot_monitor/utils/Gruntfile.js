module.exports = function(grunt) {
  var pkg = grunt.file.readJSON('package.json');
  grunt.initConfig({
    less: {
      development: {
        options: {
          paths: ['../less/']
        },
        files: {
          '../www/css/rwt_robot_monitor.css': '../less/rwt_robot_monitor.less'
        }
      },
      production: {
        options: {
          paths: ['../less/']
        },
        files: {
          '../www/css/rwt_robot_monitor.css': '../less/rwt_robot_monitor.less'
        }
      },
    },
    concat: {
      build: {
        src: ['../src/*.js'],
        dest: '../www/rwt_robot_monitor.js'
      }
    },
    uglify: {
      options: {
        report: 'min'
      },
      build: {
        src: '../www/rwt_robot_monitor.js',
        dest: '../www/rwt_robot_monitor.min.js'
      }
    },
    jshint: {
      options: {
        jshintrc: '.jshintrc'
      },
      files: [
        'Gruntfile.js',
        '../www/rwt_robot_monitor.js'
      ]
    },
    watch: {
      files: ['../src/*.js', 'Gruntfile.js', '.jshintrc', '../less/rwt_robot_monitor.less'],
      tasks: ['build', 'less', 'doc']
    },
    jsdoc: {
      dist: {
        src: ['../src/*.js'],
        options: {
          destination: '../doc'
        }
      }
    }
  });

  // reading npm tasks
  for(var taskName in pkg.devDependencies) {
    if(taskName.substring(0, 6) === 'grunt-'.toString()) {
      grunt.loadNpmTasks(taskName);
    }
  }
  
  grunt.registerTask('build', ['concat', 'jshint', 'uglify', 'less']);
  grunt.registerTask('default', ['build']);
  grunt.registerTask('doc', ['jsdoc']);
};
