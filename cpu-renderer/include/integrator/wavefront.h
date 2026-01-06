// include/integrator/wavefront.h

#pragma once

namespace COR {
	struct Film;
	struct Camera;
	struct World;

	// wavefront integrator: render exactly 1 sample for pixels inside a tile
	void render_tile_wavefront_sample(
		Film& film,
		const Camera& cam,
		const World& world,
		int W, int H,
		int tileX0, int tileY0, int tileX1, int tileY1,
		int sampleIndex,
		int maxDepth);
}