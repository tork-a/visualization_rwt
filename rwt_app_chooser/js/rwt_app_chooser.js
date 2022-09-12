var load_robots = function() {
  return store.get('robots') || [];
};

var save_robots = function(robots) {
  store.set('robots', robots);
}
