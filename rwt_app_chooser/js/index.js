phonon.options({
  navigator: {
    defaultPage: 'robot',
    animatePages: true,
    templateRootDirectory: 'components/',
    enableBrowserBackButton: true,
  },
  i18n: null
});

var app = phonon.navigator();

app.on({page: 'robot', content: 'robot.html'});

app.on({page: 'add_robot', content: 'add_robot.html'});

app.on({page: 'task', preventClose: true, content: 'task.html'});

app.start();
