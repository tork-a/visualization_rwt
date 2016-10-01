$(function() {
    var ros = new ROSLIB.Ros({
        url: 'ws://' + location.hostname + ':8888'
    });

    var topic = '/Tablet/voice';
    var tabletVoice = new ROSLIB.Topic({
        ros: ros,
        name: topic,
        messageType: 'speech_recognition_msgs/SpeechRecognitionCandidates'
    });

    var showMenuString = function (lang){
        console.log('lang selected: ' + lang);
        lang_name = $('#lang-selector li:eq(0)').text();
        for(var i = 0; i < $('#lang-selector li').length; i++) {
            if ( $('#lang-selector li:eq('+i+')').data('value') == lang ) {
                lang_name = $('#lang-selector li:eq('+i+')').text();
            }
        }
        setLanguage(lang);
        $('#continuous').text(_('continuous'));
        $('#once').text(_('once'));
        $('#speak').text(_('speak'));
        $('#language').text(_('language'));
        $('#detail-label').text(_('detail'));
        $('#lang-label').text(_('language'));
        $('#lang').text(lang_name);
        $('#status-label').text(_('status'));
        $('#clear-result').text(_('clear'));
        $('#result-label').text(_('result'));
        $('#publish-detail-label').text(_('publishdetail'));
        $('#topic-alt-button').text(_('change'));
    };

    var SpeechRecognition = window.webkitSpeechRecognition
        || window.mozSpeechRecognition
        || window.oSpeechRecognition
        || window.msSpeechRecognition
        || window.SpeechRecognition;

    if (!SpeechRecognition) $('body').html('<h1>This Browser is not supported.</h1>');

    var speech_recog = new SpeechRecognition();

    speech_recog.lang = 'ja-JP';
    speech_recog.continuous = false;
    speech_recog.interimResults = false;
    speech_recog.maxAlternatives = 5;

    speech_recog.onsoundstart = function(){
        console.log('recog start.');
        $('#status').text(_('sound start'));
    };

    speech_recog.onspeechstart = function() {
        console.log('onspeechstart');
        $('#status').text(_('speech start'));
    };

    speech_recog.onspeechend = function() {
        console.log('onspeechend');
        $('#status').text(_('speech end'));
    };
    speech_recog.onnomatch = function(){
        console.log('recog nomatch.');
        $('#status').text(_('nomatch'));
    };

    speech_recog.onerror = function(e){
        console.log('recog error.: ' + e.error);
        $('#status').text(_('error') + ': ' + e.error);
    };

    speech_recog.onsoundend = function(){
        console.log('recog soundend.');
        $('#status').text(_('soundend'));
    };

    speech_recog.onaudioend = function (){
        console.log('recog audioend.');
        if (speech_recog.continuous){
            speech_recog.stop();
            setTimeout(function(){
                speech_recog.start();
            }, 200);
        }
    };

    var addRow3 = function(col1, col2, col3){
        return '<tr><td>'+col1+'</td><td>'+col2+'</td><td>'+col3+'</td></tr>';
    };

    isPublishDetail = false;
    speech_recog.onresult = function(e){
        var recentResults = e.results[e.results.length-1];
        var texts = [];
        var confidences = [];
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
                confidences.push(conf);
            } else {
                if (recentResults.isFinal) {
                    texts.push(word);
                    confidences.push(conf);
                }
            }
        }

        table += '</table>';
        $('#messages').prepend(table);

        if (texts.length > 0){
            var msg = new ROSLIB.Message({
                transcript: texts,
                confidence: confidences
            });
            console.log('published: ' + JSON.stringify(msg));
            tabletVoice.publish(msg);
        }

        if (!speech_recog.continuous){
            console.log('speak off');
            speech_recog.stop();
            isSpeaking = false;
            $('#speak').text(_('speak'));
            $('#speak').addClass('btn-default');
        }
    };

    var isSpeaking = false;
    $('#speak').on('click', function (){
        if (!isSpeaking) {
            console.log('speak on');
            speech_recog.start();
            isSpeaking = true;
            $('#status').text(_('start recognition'));
            $('#speak').text(_('stop'));
        } else {
            console.log('speak off');
            speech_recog.stop();
            isSpeaking = false;
            $('#status').text(_('stop recognition'));
            $('#speak').text(_('speak'));
        }
    });
    $('#once').on('click', function(){
        if (speech_recog.continuous){
            $('#speak').text(_('speak')).removeAttr('disabled');
            $('#once').addClass('btn-primary');
            $('#continuous').removeClass('btn-primary');
            speech_recog.abort();
            speech_recog.continuous = false;
        }
    });
    $('#continuous').on('click', function (){
        if (!speech_recog.continuous){
            $('#speak').text(_('speak')).attr('disabled', 'disabled');
            $('#continuous').addClass('btn-primary');
            $('#once').removeClass('btn-primary');
            $('#once').addClass('btn-default');
            speech_recog.abort();
            speech_recog.continuous = true;
            speech_recog.start();
        }
    });
    $('#detail').click( function (){
        if (this.checked){
            console.log('detail enabled');
            speech_recog.abort();
            speech_recog.interimResults = true;
            speech_recog.start();
        } else {
            console.log('detail disabled');
            speech_recog.abort();
            speech_recog.interimResults = false;
            speech_recog.start();
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
        var lang = $(this).data('value');
        console.log('lang selected: ' + $(this).text() + ' ' + lang);
        showMenuString(lang);
        speech_recog.lang = lang;
        speech_recog.start();
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
                messageType: 'speech_recognition_msgs/SpeechRecognitionCandidates.msg'
            });
        }
        var alt = parseInt($('#alternative').val());
        if (alt) {
            console.log('alternative: ' + alt);
            speech_recog.maxAlternatives = alt;
        }
        speech_recog.stop();
    });

    showMenuString();
});
