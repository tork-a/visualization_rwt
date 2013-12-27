$(function() {
    var ros = new ROSLIB.Ros({
        url: 'ws://' + location.hostname + ':8888'
    });

    var topic = '/Tablet/voice';
    var tabletVoice = new ROSLIB.Topic({
        ros: ros,
        name: topic,
        messageType: 'jsk_gui_msgs/VoiceMessage'
    });

    var showMenuString = function (lang){
        setLanguage(lang);
        $('#continuous').text(_('continuous'));
        $('#once').text(_('once'));
        $('#speak').text(_('speak'));
        $('#language').text(_('language'));
        $('#detail-label').text(_('detail'));
        $('#status-label').text(_('status'));
        $('#clear-result').text(_('clear'));
        $('#result-label').text(_('result'));
        $('#publish-detail-label').text(_('publishdetail'));
        $('#topic-alt-button').text(_('change'));
    };

    var VoiceRecognition = window.webkitSpeechRecognition
        || window.mozSpeechRecognition
        || window.oSpeechRecognition
        || window.msSpeechRecognition
        || window.SpeechRecognition;

    if (!VoiceRecognition) $('body').html('<h1>This Browser is not supported.</h1>');

    var voice_recog = new VoiceRecognition();

    voice_recog.lang = 'ja-JP';
    voice_recog.continuous = false;
    voice_recog.interimResults = false;
    voice_recog.maxAlternatives = 5;

    voice_recog.onsoundstart = function(){
        console.log('recog start.');
        $('#status').text(_('soundstart'));
    };

    voice_recog.onspeechstart = function() {
        console.log('onspeechstart');
        $('#status').text(_('speechstart'));
    };

    voice_recog.onspeechend = function() {
        console.log('onspeechend');
        $('#status').text(_('speechend'));
    };
    voice_recog.onnomatch = function(){
        console.log('recog nomatch.');
        $('#status').text(_('nomatch'));
    };

    voice_recog.onerror = function(e){
        console.log('recog error.: ' + e.error);
        $('#status').text(_('error') + ': ' + e.error);
    };

    voice_recog.onsoundend = function(){
        console.log('recog soundend.');
        $('#status').text(_('soundend'));
    };

    voice_recog.onaudioend = function (){
        console.log('recog audioend.');
        if (voice_recog.continuous){
            voice_recog.stop();
            setTimeout(function(){
                voice_recog.start();
            }, 200);
        }
    };

    var addRow3 = function(col1, col2, col3){
        return '<tr><td>'+col1+'</td><td>'+col2+'</td><td>'+col3+'</td></tr>';
    };

    isPublishDetail = false;
    voice_recog.onresult = function(e){
        var recentResults = e.results[e.results.length-1];
        var texts = [];
        var table = '<table class="table table-striped table-bordered table-condenced">'
        table += addRow3(_('number'), _('word'), _('confidence'));
        console.log(e);
        console.log(recentResults);
        for (var i = 0; i < recentResults.length; ++i){
            var word = recentResults[i].transcript;
            var conf = recentResults[i].confidence;
            if (word != '')
                table += addRow3(i+1, word, conf);
            if (isPublishDetail) {
                texts.push(word);
            } else {
                if (recentResults.isFinal) texts.push(word);
            }
        }

        table += '</table>';
        $('#messages').prepend(table);

        if (texts.length > 0){
            var msg = new ROSLIB.Message({
                texts: texts
            });
            console.log('published: ' + JSON.stringify(msg));
            tabletVoice.publish(msg);
        }

        if (!voice_recog.continuous){
            console.log('speak off');
            voice_recog.stop();
            isSpeaking = false;
            $('#speak').text(_('speak'));
        }
    };

    var isSpeaking = false;
    $('#speak').on('click', function (){
        if (!isSpeaking) {
            console.log('speak on');
            voice_recog.start();
            isSpeaking = true;
            $('#speak').text(_('stop'));
        } else {
            console.log('speak off');
            voice_recog.stop();
            isSpeaking = false;
            $('#speak').text(_('speak'));
        }
    });
    $('#once').on('click', function(){
        if (voice_recog.continuous){
            $('#speak').text(_('speak')).removeAttr('disabled');
            $('#once').addClass('btn-primary');
            $('#continuous').removeClass('btn-primary');
            voice_recog.abort();
            voice_recog.continuous = false;
        }
    });
    $('#continuous').on('click', function (){
        if (!voice_recog.continuous){
            $('#speak').text(_('speak')).attr('disabled', 'disabled');
            $('#continuous').addClass('btn-primary');
            $('#once').removeClass('btn-primary');
            voice_recog.abort();
            voice_recog.continuous = true;
            voice_recog.start();
        }
    });
    $('#detail').click( function (){
        if (this.checked){
            console.log('detail enabled');
            voice_recog.abort();
            voice_recog.interimResults = true;
            voice_recog.start();
        } else {
            console.log('detail disabled');
            voice_recog.abort();
            voice_recog.interimResults = false;
            voice_recog.start();
        }
    });

    $('#publish-detail').click(function (){
        if (this.checked){
            console.log('publish detail enabled');
            isPublishDetail = true;
        } else {
            console.log('publish detail disabled');
            isPublishDetail = false;
        }
    });

    $('#lang-selector li').click(function (){
        var lang = $(this).attr('value');
        console.log('lang selected: ' + lang);
        showMenuString(lang);
        voice_recog.lang = lang;
        voice_recog.start();
    });

    $('#clear-result').click(function (){
        console.log('clear result');
        $('#messages').html('');
    });

    $('#topic-alt-button').click(function(e) {
        if (topic != $('#topic-name').val()) {
            topic = $('#topic-name').val();
            console.log('topic changed to ' + topic);
            tabletVoice = new ROSLIB.Topic({
                ros: ros,
                name: topic,
                messageType: 'jsk_gui_msgs/VoiceMessage'
            });
        }
        var alt = parseInt($('#alternative').val());
        if (alt) {
            console.log('alternative: ' + alt);
            voice_recog.maxAlternatives = alt;
        }
        voice_recog.stop();
    });

    showMenuString();
});
