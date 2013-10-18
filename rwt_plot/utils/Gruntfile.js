module.exports = function(grunt) {
  var pkg = grunt.file.readJSON('package.json');
  grunt.initConfig({
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
  });

  // reading npm tasks
  for(var taskName in pkg.devDependencies) {
    if(taskName.substring(0, 6) == 'grunt-') {
      grunt.loadNpmTasks(taskName);
    }
  }
  
  grunt.registerTask('build', ['concat', 'uglify']);
  grunt.registerTask('default', ['build']);
};
