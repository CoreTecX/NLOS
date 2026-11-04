"""Fixed-anchor similarity registration using only stable perimeter features."""
from dataclasses import dataclass
import hashlib,json
import cv2
import numpy as np
from contracts import require,sha


def transform_points(H,points):
    pts=np.asarray(points,dtype=np.float64)
    hom=np.column_stack((pts,np.ones(len(pts))))@np.asarray(H,dtype=np.float64).T
    require(np.isfinite(hom).all() and np.all(np.abs(hom[:,2])>1e-12),'DEGENERATE_TRANSFORM','Invalid projective denominator')
    return hom[:,:2]/hom[:,2:]


CORE_DOMAIN=dict(max_corner_displacement_roi_diagonal=.05,max_abs_rotation_degrees=3.,
                 max_abs_scale_delta=.05,min_overlap=.9,max_control_p95_grid_pixels=.5)


def validated_domain(domain=None):
    domain=dict(CORE_DOMAIN if domain is None else domain)
    require(set(domain)==set(CORE_DOMAIN),'MOTION_DOMAIN','Complete measured motion domain required')
    for key,limit in CORE_DOMAIN.items():
        value=domain[key]
        require(type(value) in (int,float) and np.isfinite(value),'MOTION_DOMAIN','Finite motion bound required')
        require(limit<=value<=1 if key=='min_overlap' else 0<value<=limit,
                'MOTION_DOMAIN','Measured domain may only narrow the conservative core')
    return domain


def control_errors(H,reference,current,roi):
    # Axes may have different sensor-to-measurement spacing.
    delta=(transform_points(H,reference)-current)*2
    return np.linalg.norm(delta/np.array([roi[2]/126.,roi[3]/126.]),axis=1)


def core_scope(ref_to_current,roi,domain=None):
    limits=validated_domain(domain)
    x,y,w,h=roi;corners=np.array([[x,y],[x+w,y],[x+w,y+h],[x,y+h]],np.float64)
    mapped=transform_points(ref_to_current,corners)
    displacement=float(np.max(np.linalg.norm(mapped-corners,axis=1))/np.hypot(w,h))
    linear=np.asarray(ref_to_current)[:2,:2];scale=float(np.sqrt(np.linalg.det(linear)))
    angle=float(np.degrees(np.arctan2(linear[1,0],linear[0,0])))
    area,_=cv2.intersectConvexConvex(corners.astype(np.float32),mapped.astype(np.float32))
    overlap=float(area/(w*h))
    return dict(passed=bool(displacement<=limits['max_corner_displacement_roi_diagonal']+1e-10 and
                abs(angle)<=limits['max_abs_rotation_degrees']+1e-10 and
                abs(scale-1)<=limits['max_abs_scale_delta']+1e-10 and overlap>=limits['min_overlap']-1e-10),
                max_corner_displacement_roi_diagonal=displacement,rotation_degrees=angle,scale=scale,overlap=overlap)


@dataclass(frozen=True)
class Registration:
    matrix: tuple
    frame_uid: str
    source_session_id: str
    anchor_id: str
    calibration_key: str
    sensor_shape: tuple
    diagnostics_json: str

    def __post_init__(self):
        h=np.asarray(self.matrix,dtype=np.float64)
        require(h.size==9 and np.isfinite(h).all(),'DEGENERATE_TRANSFORM','Nonfinite/malformed registration')
        h=h.reshape(3,3);a=h[:2,:2];gram=a.T@a
        require(np.allclose(h[2],[0,0,1],rtol=0,atol=1e-12) and np.linalg.det(a)>0 and
                np.allclose(gram,np.eye(2)*np.trace(gram)/2,rtol=1e-9,atol=1e-12),
                'DEGENERATE_TRANSFORM','Only a finite orientation-preserving similarity is registered')

    @property
    def current_to_reference(self):return np.array(self.matrix,dtype=np.float64).reshape(3,3)
    @property
    def diagnostics(self):return json.loads(self.diagnostics_json)
    @property
    def transform_id(self):return sha((self.frame_uid+self.anchor_id+str(self.matrix)).encode())


def tracking_gray(raw):
    """2x2 cell average; pixel (x,y) is sensor center (2x+.5,2y+.5)."""
    raw=np.asarray(raw,dtype=np.float64)
    require(raw.ndim==2 and raw.shape[0]%2==raw.shape[1]%2==0 and np.isfinite(raw).all(),'TRACKING_FORMAT','Finite even Bayer shape required')
    return (raw[0::2,0::2]+raw[0::2,1::2]+raw[1::2,0::2]+raw[1::2,1::2])*.25


class FixedAnchor:
    def __init__(self,raw,stable_mask,roi,calibration_key,policy):
        self.raw_shape=tuple(raw.shape);self.roi=tuple(roi);self.calibration_key=calibration_key;self.policy=dict(policy)
        gray=tracking_gray(raw);mask=np.asarray(stable_mask,dtype=bool)
        require(mask.shape==gray.shape and mask.sum()>=100,'ANCHOR_MASK','Stable mask must use tracking-grid coordinates')
        # Forbid fitting on the sensing ROI, including a guard for LK patches.
        x,y,w,h=roi;guard=policy.get('roi_guard_tracking_pixels',12)
        forbid=np.zeros(mask.shape,bool)
        y0=max(0,int(y/2)-guard);x0=max(0,int(x/2)-guard)
        forbid[y0:min(mask.shape[0],int((y+h)/2)+guard),x0:min(mask.shape[1],int((x+w)/2)+guard)]=True
        require(not np.any(mask&forbid),'ANCHOR_MASK','Fit mask overlaps sensing ROI/guard')
        self.low=float(policy['tracking_low']);self.high=float(policy['tracking_high'])
        require(np.isfinite(self.low) and np.isfinite(self.high) and self.high>self.low,'ANCHOR_SCALE','Fixed tracking scale required')
        self.reference=self._image(gray)
        points=cv2.goodFeaturesToTrack(self.reference,maxCorners=policy.get('max_features',240),qualityLevel=.01,minDistance=8,mask=mask.astype(np.uint8)*255)
        require(points is not None and len(points)>=30,'WEAK_REFERENCE','Not enough perimeter texture')
        points=points.reshape(-1,2)
        # Control set fixed before the current image is observed and never used in RANSAC.
        self.fit=points[np.arange(len(points))%5!=0];self.control=points[np.arange(len(points))%5==0]
        self.anchor_id=sha(np.asarray(raw,dtype='<u2').tobytes()+mask.tobytes()+str(roi).encode()+calibration_key.encode())
        self.reference=np.frombuffer(self.reference.tobytes(),np.uint8).reshape(self.reference.shape)
        self.to_sensor=np.array([[2.,0.,.5],[0.,2.,.5],[0.,0.,1.]])

    def _image(self,gray):return np.clip((gray-self.low)*255/(self.high-self.low),0,255).astype(np.uint8)

    def _track(self,current,points):
        p=points.astype(np.float32).reshape(-1,1,2)
        q,status,error=cv2.calcOpticalFlowPyrLK(self.reference,current,p,None,winSize=(21,21),maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,40,.001))
        require(q is not None and status is not None and error is not None and np.isfinite(q).all(),
                'TRACK_FAILED','Tracking returned missing/nonfinite points')
        back,back_status,_=cv2.calcOpticalFlowPyrLK(current,self.reference,q,None,winSize=(21,21),maxLevel=3)
        require(back is not None and back_status is not None,'TRACK_FAILED','Backward tracking returned no points')
        good=(status.ravel()==1)&(back_status.ravel()==1)&np.isfinite(back.reshape(-1,2)).all(axis=1)&np.isfinite(error.ravel())
        good&=np.linalg.norm(back.reshape(-1,2)-points,axis=1)<self.policy.get('forward_backward_pixels',.8)
        good&=error.ravel()<self.policy.get('lk_error_max',30.)
        return points[good].astype(np.float64),q.reshape(-1,2)[good].astype(np.float64)

    def register(self,raw,frame_uid,source_session_id,calibration_key):
        require(tuple(raw.shape)==self.raw_shape and calibration_key==self.calibration_key,'ANCHOR_CONTEXT','Configuration/calibration changed')
        current=self._image(tracking_gray(raw))
        a,b=self._track(current,self.fit)
        require(len(a)>=self.policy.get('minimum_inliers',12),'TRACK_FAILED','Insufficient stable features')
        fit,inliers=cv2.estimateAffinePartial2D(a,b,method=cv2.RANSAC,ransacReprojThreshold=.75,maxIters=2000,confidence=.999,refineIters=10)
        require(fit is not None and inliers is not None,'DEGENERATE_TRANSFORM','Similarity fit failed')
        selected=inliers.ravel().astype(bool);aa=a[selected]
        require(len(aa)>=self.policy.get('minimum_inliers',12) and selected.mean()>=.6,'TRACK_FAILED','Insufficient inliers')
        spread=np.linalg.eigvalsh(np.cov(aa.T))
        require(spread[0]>4 and spread[-1]/spread[0]<100,'DEGENERATE_TRANSFORM','Features are collinear or too local')
        H=np.vstack((fit,[0,0,1.]));det=float(np.linalg.det(H[:2,:2]))
        require(det>0 and np.linalg.cond(H[:2,:2])<2,'DEGENERATE_TRANSFORM','Invalid similarity')
        ca,cb=self._track(current,self.control)
        require(len(ca)>=6,'CONTROL_POINTS','Independent tracked controls unavailable')
        control_spread=np.linalg.eigvalsh(np.cov(ca.T))
        require(control_spread[0]>4 and control_spread[-1]/control_spread[0]<100,
                'CONTROL_POINTS','Independent controls are too local or collinear')
        errors=control_errors(H,ca,cb,self.roi)
        p95=float(np.percentile(errors,95))
        require(p95<=.5,'CONTROL_ERROR','Independent control error exceeds half a measurement pixel')
        sensorH=self.to_sensor@H@np.linalg.inv(self.to_sensor)
        scope=core_scope(sensorH,self.roi)
        require(scope['passed'],'OUTSIDE_CAM_CORE','Registered CAM_CORE joint boundary exceeded')
        diagnostics=dict(fit_points=len(a),inliers=int(selected.sum()),independent_controls=len(ca),
            independent_control_p95_grid_pixels=p95,scope=scope,fixed_anchor=True,control_used_in_fit=False,
            physical_model_equivalence_proven=False)
        return Registration(tuple(np.linalg.inv(sensorH).ravel()),frame_uid,source_session_id,self.anchor_id,
                            calibration_key,self.raw_shape,json.dumps(diagnostics,allow_nan=False))
