(function(func) {
  if (typeof exports == "object") {
    module.exports = func();
  } else if (typeof define === "function" && define.amd) {
    define(func);
  } else {
    RWTSpeechRecognition = func();
  }
})(function(){
  'use strict';

  var RWTSpeechRecognition = function(language, dialect) {
    this.languages = [
      ['Afrikaans',        ['af-ZA']],
      ['አማርኛ',             ['am-ET']],
      ['Azərbaycanca',     ['az-AZ']],
      ['বাংলা',            ['bn-BD', 'বাংলাদেশ'],
                           ['bn-IN', 'ভারত']],
      ['Bahasa Indonesia', ['id-ID']],
      ['Bahasa Melayu',    ['ms-MY']],
      ['Català',           ['ca-ES']],
      ['Čeština',          ['cs-CZ']],
      ['Dansk',            ['da-DK']],
      ['Deutsch',          ['de-DE']],
      ['English',          ['en-AU', 'Australia'],
                           ['en-CA', 'Canada'],
                           ['en-IN', 'India'],
                           ['en-KE', 'Kenya'],
                           ['en-TZ', 'Tanzania'],
                           ['en-GH', 'Ghana'],
                           ['en-NZ', 'New Zealand'],
                           ['en-NG', 'Nigeria'],
                           ['en-ZA', 'South Africa'],
                           ['en-PH', 'Philippines'],
                           ['en-GB', 'United Kingdom'],
                           ['en-US', 'United States']],
      ['Español',          ['es-AR', 'Argentina'],
                           ['es-BO', 'Bolivia'],
                           ['es-CL', 'Chile'],
                           ['es-CO', 'Colombia'],
                           ['es-CR', 'Costa Rica'],
                           ['es-EC', 'Ecuador'],
                           ['es-SV', 'El Salvador'],
                           ['es-ES', 'España'],
                           ['es-US', 'Estados Unidos'],
                           ['es-GT', 'Guatemala'],
                           ['es-HN', 'Honduras'],
                           ['es-MX', 'México'],
                           ['es-NI', 'Nicaragua'],
                           ['es-PA', 'Panamá'],
                           ['es-PY', 'Paraguay'],
                           ['es-PE', 'Perú'],
                           ['es-PR', 'Puerto Rico'],
                           ['es-DO', 'República Dominicana'],
                           ['es-UY', 'Uruguay'],
                           ['es-VE', 'Venezuela']],
      ['Euskara',          ['eu-ES']],
      ['Filipino',         ['fil-PH']],
      ['Français',         ['fr-FR']],
      ['Basa Jawa',        ['jv-ID']],
      ['Galego',           ['gl-ES']],
      ['ગુજરાતી',           ['gu-IN']],
      ['Hrvatski',         ['hr-HR']],
      ['IsiZulu',          ['zu-ZA']],
      ['Íslenska',         ['is-IS']],
      ['Italiano',         ['it-IT', 'Italia'],
                           ['it-CH', 'Svizzera']],
      ['ಕನ್ನಡ',             ['kn-IN']],
      ['ភាសាខ្មែរ',         ['km-KH']],
      ['Latviešu',         ['lv-LV']],
      ['Lietuvių',         ['lt-LT']],
      ['മലയാളം',           ['ml-IN']],
      ['मराठी',            ['mr-IN']],
      ['Magyar',           ['hu-HU']],
      ['ລາວ',              ['lo-LA']],
      ['Nederlands',       ['nl-NL']],
      ['नेपाली भाषा',       ['ne-NP']],
      ['Norsk bokmål',     ['nb-NO']],
      ['Polski',           ['pl-PL']],
      ['Português',        ['pt-BR', 'Brasil'],
                           ['pt-PT', 'Portugal']],
      ['Română',           ['ro-RO']],
      ['සිංහල',             ['si-LK']],
      ['Slovenščina',      ['sl-SI']],
      ['Basa Sunda',       ['su-ID']],
      ['Slovenčina',       ['sk-SK']],
      ['Suomi',            ['fi-FI']],
      ['Svenska',          ['sv-SE']],
      ['Kiswahili',        ['sw-TZ', 'Tanzania'],
                           ['sw-KE', 'Kenya']],
      ['ქართული',          ['ka-GE']],
      ['Հայերեն',          ['hy-AM']],
      ['தமிழ்',             ['ta-IN', 'இந்தியா'],
                           ['ta-SG', 'சிங்கப்பூர்'],
                           ['ta-LK', 'இலங்கை'],
                           ['ta-MY', 'மலேசியா']],
      ['తెలుగు',            ['te-IN']],
      ['Tiếng Việt',       ['vi-VN']],
      ['Türkçe',           ['tr-TR']],
      ['اُردُو',             ['ur-PK', 'پاکستان'],
                           ['ur-IN', 'بھارت']],
      ['Ελληνικά',         ['el-GR']],
      ['български',        ['bg-BG']],
      ['Pусский',          ['ru-RU']],
      ['Српски',           ['sr-RS']],
      ['Українська',       ['uk-UA']],
      ['한국어',           ['ko-KR']],
      ['中文',             ['cmn-Hans-CN', '普通话 (中国大陆)'],
                           ['cmn-Hans-HK', '普通话 (香港)'],
                           ['cmn-Hant-TW', '中文 (台灣)'],
                           ['yue-Hant-HK', '粵語 (香港)']],
      ['日本語',           ['ja-JP']],
      ['हिन्दी',            ['hi-IN']],
      ['ภาษาไทย',          ['th-TH']]
    ];

    this.continuous = false;
    this.recognizing = false;
    this.stopping = false;
    this.ignore_on_end = false;
    this.hooks = {};
    this.sr = null;
    
    this.sel_lang = document.getElementById(language);
    this.sel_dial = document.getElementById(dialect);
    for (var i = 0; i < this.languages.length; i++) {
      this.sel_lang.options.add(new Option(this.languages[i][0], i));
    }
    
    // default lang
    this.sel_lang.selectedIndex = 10; // English
    this.updateDialect();
    this.sel_dial.selectedIndex = 11; // US
    // callbacks
    this.sel_lang.onchange = this.updateDialect.bind(this);
    var that = this;
    this.sel_dial.onchange = function() {
      if (that.recognizing) {
        that.restart();
      }
    };
  };

  RWTSpeechRecognition.prototype.getRecognizer = function() {
    return window.webkitSpeechRecognition
        || window.mozSpeechRecognition
        || window.oSpeechRecognition
        || window.msSpeechRecognition
        || window.SpeechRecognition;
  };

  RWTSpeechRecognition.prototype.isSupported = function() {
    return !(!this.getRecognizer());
  };

  RWTSpeechRecognition.prototype.updateDialect = function(e) {
    for (var i = this.sel_dial.options.length - 1; i >= 0; i--) {
      this.sel_dial.remove(i);
    }
    var list = this.languages[this.sel_lang.selectedIndex];
    for (var i = 1; i < list.length; i++) {
      this.sel_dial.options.add(new Option(list[i][1], list[i][0]));
    }
    this.sel_dial.disabled = list[1].length == 1;
  };

  RWTSpeechRecognition.prototype.setContinuous = function(enable) {
    if (enable && !this.continuous) {
      this.stop();
      this.continuous = true;
    } else if (!enable && this.continuous) {
      this.stop();
      this.continuous = false;
    }
  };

  RWTSpeechRecognition.prototype.on = function(event, cb) {
    this.hooks[event] = cb;
  };

  RWTSpeechRecognition.prototype.start = function() {
    var that = this;
    this.sr = new (this.getRecognizer())();
    this.sr.lang = this.sel_dial.value;
    this.sr.continuous = this.continuous;
    this.sr.interimResults = true;
    this.sr.maxAlternatives = 5;
    this.sr.onstart = function() {
      that.recognizing = true;
      if ("start" in that.hooks) {
        that.hooks["start"]();
      }
    };
    this.sr.onerror = function(event) {
      if (event.error == 'no-speech'     ||
          event.error == 'audio-capture' ||
          event.error == 'not-allowed') {
        that.ignore_on_end = true;
      }

      that.recognizing = false;
      if ("error" in that.hooks) {
        that.hooks["error"](event);
      }
    };
    this.sr.onend = function() {
      that.recognizing = false;
      if (that.ignore_on_end) { return; }
      if ("end" in that.hooks) {
        that.hooks["end"]();
      }
    };
    this.sr.onresult = function(event) {
      if (typeof(event.results) == 'undefined') {
        that.stop();
        return;
      }

      var isFinal = false;
      var interim = "";
      var finals = [];

      for (var i = event.resultIndex; i < event.results.length; i++) {
        if (event.results[i].isFinal) {
          isFinal = true;
          while (event.results[i].length > finals.length) {
            finals.push({transcript: "", confidence: 0.0, n: 0});
          }
          for (var j = 0; j < event.results[i].length; j++) {
            finals[j].transcript += event.results[i][j].transcript;
            finals[j].confidence += event.results[i][j].confidence;
            finals[j].n += 1;
          }
        } else {
          interim += event.results[i][0].transcript;
        }
      }

      for (var i = 0; i < finals.length; i++) {
        finals[i].confidence *= 1.0 / finals[i].n;
      }
      
      if (!isFinal) {
        finals = [];
      }

      finals.sort(function(a, b) {
        return b.confidence - a.confidence;
      });
      
      if ("result" in that.hooks) {
        that.hooks["result"](finals, interim);
      }
    };
    this.sr.onaudioend = function() {
      if (that.continuous && !that.stopping) {
        that.restart();
      }
      that.stopping = false;
    };
    this.sr.start();
  };

  RWTSpeechRecognition.prototype.stop = function() {
    if (this.sr) {
      this.stopping = true;
      this.sr.stop();
    }
  };

  RWTSpeechRecognition.prototype.restart = function() {
    this.stop();
    var that = this;
    setTimeout(function() {
      that.start();
    }, 200);
  };
  return RWTSpeechRecognition;
});
