use dhcp::Dhcp;
use w5500_ll::net::Eui48Addr;

#[test]
fn discover() {
    const XID: u32 = 0x12345678;
    let mut buf = unsafe { Dhcp::steal() };
    buf.dhcp_discover(
        &Eui48Addr::new(0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC),
        "HOSTNAME",
        &XID,
    );

    assert!(!buf.is_bootreply());
    assert!(buf.is_bootrequest());
    assert!(buf.is_htype_ethernet());
    assert_eq!(buf.xid(), XID);
}
