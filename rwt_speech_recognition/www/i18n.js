(function(window, undefined) {
    window.__msgStore = {};
    window.__persistMsgStore = function(lang, data) {
        if(window.localStorage) {
            localStorage.setItem('localizationMsgStore'+lang, JSON.stringify(data));
            window.__msgStore = data;
        } else {
            window.__msgStore = data;
        }
    };
    window.__getLanguageJSON = function(lang) {
        var res = false;
        $.ajax({
            async: false,
            url: 'locale/' + lang + '.json',
            dataType: 'json',
            success: function (json){
                console.log('get json success');
                res = true;
            }
        });
        return res;
    };

    window.setLanguage = function(l) {
        var lang = l || 'ja-JP';

        if(window.localStorage) {
            var localMsgStore = localStorage.getItem('localizationMsgStore'+lang);
            if(localMsgStore) {
                console.log('use local cache');
                window.__msgStore = JSON.parse(localMsgStore);
            } else {
                window.__getLanguageJSON(lang);
            }
        } else {
            window.__getLanguageJSON(lang);
        }
    };

    window._ = function (key){
        return window.__msgStore[key] || key;
    };

})(window);
