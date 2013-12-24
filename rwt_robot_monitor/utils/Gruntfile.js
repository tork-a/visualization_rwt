module.exports = function(grunt) {
  var pkg = grunt.file.readJSON('package.json');
  grunt.initConfig({
    less: {
      development: {
        options: {
          paths: ['../less/']
        },
        files: {
          '../www/css/rwt_plot.css': '../less/rwt_plot.less'
        }
      },
      production: {
        options: {
          paths: ['../less/']
        },
        files: {
          '../www/css/rwt_plot.css': '../less/rwt_plot.less'
        }
      },
    },
    concat: {
      build: {
        src: ['../src/*.js'],
        dest: '../www/rwt_plot.js'
      }
    },
    uglify: {
      options: {
        report: 'min'
      },
      build: {
        src: '../www/rwt_plot.js',
        dest: '../www/rwt_plot.min.js'
      }
    },
    jshint: {
      options: {
        jshintrc: '.jshintrc'
      },
      files: [
        'Gruntfile.js',
        '../www/rwt_plot.js'
      ]
    },
    watch: {
      files: ['../src/*.js', 'Gruntfile.js', '.jshintrc', '../less/rwt_plot.less'],
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
