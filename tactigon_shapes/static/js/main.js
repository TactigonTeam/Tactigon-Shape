const sidebar_resize = () => {
    const navbars_h = $("body>nav").map((e, i)=>{
        return i.offsetHeight;
    }).toArray().reduce((a,b)=> a+b, 0);

    const windows_h = $(window).height();
    const sidebar = $(".sidebar");
    const siblings_h = sidebar.siblings().map((e, i)=>{return i.offsetHeight;}).toArray();
    const sibling_h = Math.max(...siblings_h);
    const sidebar_h = windows_h-navbars_h;

    $(".sidebar").outerHeight(sidebar_h > sibling_h ? sidebar_h : sibling_h);
}

function round(value, precision) {
    var multiplier = Math.pow(10, precision || 0);
    return Math.round(value * multiplier) / multiplier;
}

function convertRange( value, r1, r2 ) { 
    return ( value - r1[ 0 ] ) * ( r2[ 1 ] - r2[ 0 ] ) / ( r1[ 1 ] - r1[ 0 ] ) + r2[ 0 ];
}

function show_tskin(tskin, connected){
    if (connected){
        tskin.find(".battery-container").removeClass("d-none");
        tskin.find(".connected").removeClass("d-none");
        tskin.find(".disconnected").addClass("d-none");
    } else {
        tskin.find(".battery-container").addClass("d-none");
        tskin.find(".connected").addClass("d-none");
        tskin.find(".disconnected").removeClass("d-none");
    }
}

const update_tskin_status = (tskin, data) => {
    if (data === undefined){
        show_tskin(tskin, false)
        return;
    }

    if (last_connection_status != data.connected) {
        last_connection_status = data.connected;

        show_tskin(tskin, data.connected)
    }

    if (data.connected) {
        if (last_battery_update_ts == 0 || Date.now()-last_battery_update_ts > BATTERY_REFRESH_RATE ){

            last_battery_update_ts = Date.now();
           
            let batt = Math.round(convertRange(round(data.battery, 2), [3.2, 4.2], [0, 100]));

            if (batt < 1) {
                batt = 0;
            }
            if (batt > 100) {
                batt = 100;
            }

            last_battery_update_value = batt;

            const batt_icon = "M2 6h" + Math.round(batt/10) + "v4H2z";
    
            tskin.find("#battery-voltage").html(batt + "%");
            tskin.find("#battery-perc").attr("d", batt_icon)
        }
    }
}

const update_braccio_status = (braccio, data) => {
    if (data === undefined){
        show_tskin(tskin, false)
        return;
    }

    if (last_braccio_connection_status != data.braccio_connection) {
        last_braccio_connection_status = data.braccio_connection;

        if (data.braccio_connection){
            braccio.find(".connected").removeClass("d-none");
            braccio.find(".disconnected").addClass("d-none");
        } else {
            braccio.find(".connected").addClass("d-none");
            braccio.find(".disconnected").removeClass("d-none");
        }
    }
}

const socket = io();
const BATTERY_REFRESH_RATE = 1*2*1000;
var last_battery_update_ts = 0;
var last_battery_update_value = 0;
var last_connection_status = undefined;
var last_braccio_connection_status = undefined;

$(()=>{
    /*
    sidebar_resize()

    $(window).on("resize", ()=>{
        sidebar_resize()
    });
    */
    
    $('.toast').map((i, el) => {
        let toast_option = {
            animation: true,
            autohide: true,
            delay: 3000
        };

        if (el.classList.contains("danger")){
            toast_option.autohide = false;
        }

        new bootstrap.Toast(el, toast_option).show();
    });

    const tskin = $("#tskin-management");
    const braccio = $("#braccio-management");
    
    $("a").click(function(){
        const loading_msg = $(this).attr("loading-msg");
        $("#loading_message").html(loading_msg == undefined ? "" : loading_msg);
        $("body").addClass("on_loading");
    });

    socket.on("state", function(data) {
        update_tskin_status(tskin, data);
        update_braccio_status(braccio, data);
    })
})