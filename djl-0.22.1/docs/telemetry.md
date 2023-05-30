# Usage Tracking

Since DJL 0.20.0, DJL collects telemetry to help us better understand our users’ needs
when running on AWS EC2. DJL contains code that allows DJL development team to collect the
AWS EC2 instance-type, instance-id and DJL version information. No other information about
the system is collected or retained.

To opt out of usage tracking for DJL, you can set the `OPT_OUT_TRACKING` environment variable:

```
export OPT_OUT_TRACKING=true
```

or Java System property:

```
System.setProperty("OPT_OUT_TRACKING", "true")
```

Usage tracking is also disable in `offline` mode:

```
System.setProperty("offline", "true")
```
