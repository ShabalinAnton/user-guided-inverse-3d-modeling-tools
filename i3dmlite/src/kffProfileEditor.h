#ifndef KFF_PROF_EDITOR_H
#define KFF_PROF_EDITOR_H

#include "mesh.h"

namespace i3dm {

static double thresholdForHelix = .01;

// the profile editor transforms a selection on a triangle mesh into a 2d profile that the user can edit
// it's used by the stationary sweep module
struct ProfileEditor {
    

    TriangleMesh *src;
    struct Ray { vec3 start, dir; };
    Ray axis;
    vector<double> kffParams;
    vec3 lcc, lcc_bar;
    //LinearComplex lc;
    vec3 refpt;
    vec3 refbase;

    mat3 xformPts;

    mat3 xformPtsInv;

    vec3 X, Y; // normalized
    double magc, spd;
    vec3 vel;
    vec3 dir;


    

    vec2 center;
    double scale;
    bool active;

    struct V {
        double angoff, t;
        vec2 pos;
        int parent;
        int num;
        vector<size_t> nbrs;
        bool on;

        V() : on(true) { nbrs.reserve(10); }
    };

    struct T {
        int ind[3];
    };

    vector<V> verts;
    vector<T> tris;

    ProfileEditor() : src(NULL), center(0,0), scale(1), randoffScale(0), active(false) {}

	void clear() {
		src = NULL; scale = 1; active = false;
	}

    ProfileEditor(TriangleMesh *mesh, Ray &axis, vector<double> &kffParams, vec3 refpt, mat3 scaleMat) : center(0,0), scale(1), randoffScale(0), active(false) {
		load(mesh, axis, kffParams, refpt, scaleMat);
	}

    V proj(int parent, vec3 &p2) {
        V toret;
        vec3 p = xformPts*p2;
        
        vec3 YY = lcc%(p-axis.start);
        double angoff = atan2(YY*Y,YY*X)-M_PI*.5;
        double yi = (dir*(p-axis.start)); // initial y of 2d projection
        vec3 base = yi*dir + axis.start;
        double xi = (p-base).length();
        double t = -angoff/magc;
        double numd = (refpt-base-vel*t)*dir/(spd*2*M_PI/magc);
        int num = (int)(numd+.5);
        if (numd<0) num = (int)(numd-.5);
        if (fabs(spd) < thresholdForHelix) {
            num = 0;
            vel = vec3(0);
        }
        t += num*2*M_PI/magc;

        double yoff = dir*(base+vel*t-refbase);
        toret.pos = vec2(xi, yoff);
        toret.t = t;
        toret.angoff = angoff;
        toret.parent = parent;
        toret.num = num;

        V v = toret;
        vec3 toret2 = refbase + X*(toret.pos[0]*cos(-v.t*magc))+Y*(toret.pos[0]*sin(-v.t*magc))-vel*v.t + toret.pos[1]*dir;
        p = (v.pos/scale) + center;
        vec3 toret3 = refbase + X*(p[0]*cos(-v.t*magc))+Y*(p[0]*sin(-v.t*magc))-vel*v.t + p[1]*dir;
        //cout << toret2 << " " << toret3 << endl;
        //genericfout << " --- " << endl << p << endl << toret2 << endl;// << toret3 << endl << toret4 << endl;
        //genericfout << "{{ " << parent << src->vertices[parent].p << " -- " << angoff << " -- " << t << " -- " << toret.pos[1] << " -- " << toret.pos[0] << " -- " << magc << endl;
        //genericfout << "b: " << p << " -- " << magc << " -- " << t << " -- " << lcc << " -- " << axis.start << " -- " << base << " -- " <<  angoff << endl;
        //genericfout << toret.pos << " -- " << toret.t << " -- " << toret.angoff << " -- " << toret.parent << endl;
        return toret;
        // untested, but I think this function is pretty much complete.
    }
    vec3 unproj(V &v) { 
        vec2 p = (v.pos/scale) + center;
        vec3 toret = refbase + X*(p[0]*cos(-v.t*magc))+Y*(p[0]*sin(-v.t*magc))-vel*v.t + p[1]*dir;
        //genericfout << toret << endl;
        //genericfout << "|| " << v.parent << src->vertices[v.parent].p << " -- " << v.angoff << " -- " << v.t << " -- " << p[1] << " -- " << p[0] << " -- " << magc << endl;
        
        return xformPtsInv*toret;
    }
    vec2 unproj_p(V &v) {
        return (v.pos/scale) + center;
    }

    void load(TriangleMesh *mesh, Ray axisi, vector<double> &kfpi, vec3 refpti, mat3 xformPts = identity2D()) {
        lcc = vec3(kfpi[0], kfpi[1], kfpi[2]);
        lcc_bar = vec3(kfpi[3], kfpi[4], kfpi[5]);

        active = true;
        axisi.start = axisi.start;
        axisi.dir = axisi.dir;

		this->axis = axisi;
        this->kffParams = kfpi;
        this->xformPts = xformPts;
        this->xformPtsInv = xformPts.inverse();
        this->refpt = xformPts * refpti;

        
        verts.reserve(mesh->vertices.size());
        
        verts.clear();
        tris.clear();
        

        src = mesh;
        if (!mesh) return;


        dir = lcc;
        dir.normalize();

        
        
        vec3 away = refpt - axis.start;
        if ((dir % away).length2() < .0000001) {
            away = vec3(-1,2,3);
            refpt = away+axis.start;
        }
        Y = dir%away;
        X = Y%dir;
        Y.normalize(); X.normalize();
        vel = (lcc_bar) + (lcc%axis.start);
        spd = vel*dir;
        magc = lcc.length();

        this->refbase = ((refpt-axis.start)*dir)*dir + axis.start;

        
		
		for (size_t i = 0; i < mesh->vertices.size(); i++) {
			mesh->vertexTags[i] = -1;
		}
        for (size_t i = 0; i < mesh->triangles.size(); i++) {
            Tri &t = mesh->triangles[i];
            if (mesh->triangleTags[i] == mesh->activeTag) {
                for (int ii = 0; ii < 3; ii++) {
                    mesh->vertexTags[t.ind[ii]] = 0;
                }
            }
        }

        

//        ofstream ofi("testout2.txt");
        //genericfout.open("testout3.txt");
		for (size_t i = 0; i < mesh->vertices.size(); i++) {
			//Tri &t = mesh->triangles[i];
			if (mesh->vertexTags[i] == 0) {
				//for (int ii = 0; ii < 3; ii++) {
					//if (mesh->vertices[i].temp == -1) {
						mesh->vertexTags[i] = (int)verts.size();
                        //ofi << mesh->vertices[t.ind[ii]].p << endl;
						verts.push_back(proj((int)i, mesh->vertices[i]));
                        //unproj(verts.back());
                        //ofi << "b: " << verts.back().pos << endl;
                        //ofi << "c: " << lcc << " -- " << lcc_bar << " -- " << lc.gamma << endl;
					//}
				//}
			}
		}
        for (size_t i = 0; i < mesh->triangles.size(); i++) {
            Tri &tri = mesh->triangles[i];
            T t;
            for (int ii = 0; ii < 3; ii++) {
                t.ind[ii] = mesh->vertexTags[tri.ind[ii]];
            }
            tris.push_back(t);
        }

        
			
        for (size_t i = 0; i < mesh->triangles.size(); i++) {
            Tri &t = mesh->triangles[i];
            if (mesh->triangleTags[i] == mesh->activeTag) {
				for (int ii = 0; ii < 3; ii++) {
					for (int jj = 0; jj < 3; jj++) {
                        if (ii != jj) {
							verts[mesh->vertexTags[t.ind[ii]]].nbrs.push_back(mesh->vertexTags[t.ind[jj]]);
                        }
					}
				}
			}
        }

        scaleToBox();

    }

	void scaleToBox() {
        if (verts.empty()) return;
		vec2 start = verts[0].pos;
		vec2 ma = start, mi = start;
		for (size_t i = 0; i < verts.size(); ++i) {
			ma = max(verts[i].pos, ma);
			mi = min(verts[i].pos, mi);
		}
		center = (ma+mi)*.5;
		scale = 1.0/max(ma[0]-mi[0], ma[1]-mi[1]);

		for (size_t i = 0; i < verts.size(); i++) {
			verts[i].pos = (verts[i].pos-center)*scale;
		}
	}

	double randoffScale;

	void draw(int lineWidth = 4) {
		if (!src)
			return;

        /*glBegin(GL_TRIANGLES);
		srand(0);
		for (size_t i = 0; i < tris.size(); i++) {
			
			//vec2 randoff(rand()%100, rand()%100);
			//randoff *= randoffScale;
            bool skiptri = false;
            for (int ii = 0; ii < 3; ii++) {
                if ( (tris[i].ind[ii] < 0) )//|| (!verts[tris[i].ind[ii]].on) )
					skiptri = true;
            }
            if (skiptri) continue;
            for (int ii = 0; ii < 3; ii++) {
                V &v = verts[tris[i].ind[ii]];
                if (v.on) {
                    glColor3d(0,1,0);
                } else {
                    glColor3d(1,.5,0);
                }
                glVertex2dv(&v.pos[0]);
            }
		}
		glEnd();*/

		glLineWidth((float)lineWidth);
        glBegin(GL_LINES);
		for (size_t i = 0; i < tris.size(); i++) {
			
			//vec2 randoff(rand()%100, rand()%100);
			//randoff *= randoffScale;
            bool skiptri = false;
            for (int ii = 0; ii < 3; ii++) {
                if ( (tris[i].ind[ii] < 0) )//|| (!verts[tris[i].ind[ii]].on))
                    skiptri = true;
            }
            if (skiptri) continue;
            for (int ii = 0; ii < 3; ii++) {
                int jj = (ii+1)%3;
                V &v = verts[tris[i].ind[ii]];
                if (v.on) {
                    glColor3d(0,1,0);
                } else {
                    glColor3d(1,.5,0);
                }
                vec2 vp = v.pos;
                vec2 vo = verts[tris[i].ind[jj]].pos;
                double hstep = scale*(dir*vel)*2*M_PI/magc;
                if (fabs(spd) >= thresholdForHelix) {
                    if (fabs(vp[1] - vo[1]) > 
                        fabs(vp[1]+hstep - vo[1]) )
                    {
                        vp[1] += hstep;
                    }
                    else if (fabs(vp[1] - vo[1]) > fabs(vp[1] - (vo[1]+hstep)) )
                    {
                        vo[1] += hstep;
                    }
                }
                glVertex2dv(&vp[0]);
                glVertex2dv(&vo[0]);
            }
		}
		glEnd();
	}

    void toggleOnOff(vec2 p, double dsq, bool on) {
        for (size_t i = 0; i < src->vertices.size(); i++) {
            src->vertexTags[i] = -1;
        }

        for (size_t i = 0; i < verts.size(); i++) {
            vec2 diff = (verts[i].pos-p);
            if (diff*diff < dsq) {
                verts[i].on = on;
            }
            src->vertexTags[verts[i].parent] = -(!verts[i].on);
        }
        
        for (size_t i = 0; i < src->triangles.size(); i++) {
            int onc = 0;
            for (int ii = 0; ii < 3; ii++) {
                onc += (src->vertexTags[src->triangles[i].ind[ii]] == 0);
            }
            src->triangleTags[i] = -(!(onc==3));
        }
    }

	void move(vec2 p, vec2 d, double dsq, bool falloff = true) {
		for (size_t i = 0; i < verts.size(); i++) {
            if (!verts[i].on)
                continue;

			vec2 diff = (verts[i].pos-p);
			if (diff*diff < dsq) {
				double wt = 1;
				if (falloff) {
					wt = 1-(diff*diff)/dsq;
				}
				verts[i].pos += d*wt;
			}
		}
	}

    void pushParents() {
		double alen = axis.dir.length();
		axis.dir /= alen;
		for (size_t vi = 0; vi < verts.size(); vi++) {
			vec2 pos = center+(verts[vi].pos/scale);
            src->vertices[verts[vi].parent] = unproj(verts[vi]);
		}
		src->computeNormals();
	}

};

} // namespace i3dm

#endif

