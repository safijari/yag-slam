#ifndef CORRELATION_GRID_H
#define CORRELATION_GRID_H

#include "Grid.h"
#include "Rectangle2.h"

class CorrelationGrid : public Grid<uint8_t> {
public:
  /**
   * Destructor
   */
  virtual ~CorrelationGrid() { delete[] m_pKernel; }

public:
  /**
   * Create a correlation grid of given size and parameters
   * @param width
   * @param height
   * @param resolution
   * @param smearDeviation
   * @return correlation grid
   */
  static CorrelationGrid *CreateGrid(int32_t width, int32_t height,
                                     double resolution, double smearDeviation) {
    assert(resolution != 0.0);

    // +1 in case of roundoff
    uint32_t borderSize = GetHalfKernelSize(smearDeviation, resolution) + 1;

    CorrelationGrid *pGrid = new CorrelationGrid(width, height, borderSize,
                                                 resolution, smearDeviation);

    return pGrid;
  }

  /**
   * Gets the index into the data pointer of the given grid coordinate
   * @param rGrid
   * @param boundaryCheck
   * @return grid index
   */
  virtual int32_t GridIndex(const Vector2<int32_t> &rGrid,
                            bool boundaryCheck = true) const {
    int32_t x = rGrid.GetX() + m_Roi.GetX();
    int32_t y = rGrid.GetY() + m_Roi.GetY();

    return Grid<uint8_t>::GridIndex(Vector2<int32_t>(x, y), boundaryCheck);
  }

  /**
   * Get the Region Of Interest (ROI)
   * @return region of interest
   */
  inline const Rectangle2<int32_t> &GetROI() const { return m_Roi; }

  /**
   * Sets the Region Of Interest (ROI)
   * @param roi
   */
  inline void SetROI(const Rectangle2<int32_t> &roi) { m_Roi = roi; }

  /**
   * Smear cell if the cell at the given point is marked as "occupied"
   * @param rGridPoint
   */
  inline void SmearPoint(const Vector2<int32_t> &rGridPoint) {
    assert(m_pKernel != NULL);

    int gridIndex = GridIndex(rGridPoint);
    if (GetDataPointer()[gridIndex] != GridStates_Occupied) {
      return;
    }

    int32_t halfKernel = m_KernelSize / 2;

    // apply kernel
    for (int32_t j = -halfKernel; j <= halfKernel; j++) {
      uint8_t *pGridAdr = GetDataPointer(
          Vector2<int32_t>(rGridPoint.GetX(), rGridPoint.GetY() + j));

      int32_t kernelConstant = (halfKernel) + m_KernelSize * (j + halfKernel);

      // if a point is on the edge of the grid, there is no problem
      // with running over the edge of allowable memory, because
      // the grid has margins to compensate for the kernel size
      for (int32_t i = -halfKernel; i <= halfKernel; i++) {
        int32_t kernelArrayIndex = i + kernelConstant;

        uint8_t kernelValue = m_pKernel[kernelArrayIndex];
        if (kernelValue > pGridAdr[i]) {
          // kernel value is greater, so set it to kernel value
          pGridAdr[i] = kernelValue;
        }
      }
    }
  }

protected:
  /**
   * Constructs a correlation grid of given size and parameters
   * @param width
   * @param height
   * @param borderSize
   * @param resolution
   * @param smearDeviation
   */
  CorrelationGrid(uint32_t width, uint32_t height, uint32_t borderSize,
                  double resolution, double smearDeviation)
      : Grid<uint8_t>(width + borderSize * 2, height + borderSize * 2),
        m_SmearDeviation(smearDeviation), m_pKernel(NULL) {
    GetCoordinateConverter()->SetScale(1.0 / resolution);

    // setup region of interest
    m_Roi = Rectangle2<int32_t>(borderSize, borderSize, width, height);

    // calculate kernel
    CalculateKernel();
  }

  /**
   * Sets up the kernel for grid smearing.
   */
  virtual void CalculateKernel() {
    double resolution = GetResolution();

    assert(resolution != 0.0);
    assert(m_SmearDeviation != 0.0);

    // min and max distance deviation for smearing;
    // will smear for two standard deviations, so deviation must be at least 1/2
    // of the resolution
    const double MIN_SMEAR_DISTANCE_DEVIATION = 0.5 * resolution;
    const double MAX_SMEAR_DISTANCE_DEVIATION = 10 * resolution;

    // check if given value too small or too big
    if (!amath::InRange(m_SmearDeviation, MIN_SMEAR_DISTANCE_DEVIATION,
                       MAX_SMEAR_DISTANCE_DEVIATION)) {
      std::stringstream error;
      error << "Mapper Error:  Smear deviation too small:  Must be between "
            << MIN_SMEAR_DISTANCE_DEVIATION << " and "
            << MAX_SMEAR_DISTANCE_DEVIATION;
      throw std::runtime_error(error.str());
    }

    // NOTE:  Currently assumes a two-dimensional kernel

    // +1 for center
    m_KernelSize = 2 * GetHalfKernelSize(m_SmearDeviation, resolution) + 1;

    // allocate kernel
    m_pKernel = new uint8_t[m_KernelSize * m_KernelSize];
    if (m_pKernel == NULL) {
      throw std::runtime_error("Unable to allocate memory for kernel!");
    }

    // calculate kernel
    int32_t halfKernel = m_KernelSize / 2;
    for (int32_t i = -halfKernel; i <= halfKernel; i++) {
      for (int32_t j = -halfKernel; j <= halfKernel; j++) {
#ifdef WIN32
        double distanceFromMean = _hypot(i * resolution, j * resolution);
#else
        double distanceFromMean = hypot(i * resolution, j * resolution);
#endif
        double z = exp(-0.5 * pow(distanceFromMean / m_SmearDeviation, 2));

        uint32_t kernelValue =
            static_cast<uint32_t>(amath::Round(z * GridStates_Occupied));
        assert(amath::IsUpTo(kernelValue, static_cast<uint32_t>(255)));

        int kernelArrayIndex =
            (i + halfKernel) + m_KernelSize * (j + halfKernel);
        m_pKernel[kernelArrayIndex] = static_cast<uint8_t>(kernelValue);
      }
    }
  }

  /**
   * Computes the kernel half-size based on the smear distance and the grid
   * resolution. Computes to two standard deviations to get 95% region and to
   * reduce aliasing.
   * @param smearDeviation
   * @param resolution
   * @return kernel half-size based on the parameters
   */
  static int32_t GetHalfKernelSize(double smearDeviation, double resolution) {
    assert(resolution != 0.0);

    return static_cast<int32_t>(amath::Round(2.0 * smearDeviation / resolution));
  }

private:
  /**
   * The point readings are smeared by this value in X and Y to create a
   * smoother response. Default value is 0.03 meters.
   */
  double m_SmearDeviation;

  // Size of one side of the kernel
  int32_t m_KernelSize;

  // Cached kernel for smearing
  uint8_t *m_pKernel;

  // region of interest
  Rectangle2<int32_t> m_Roi;
}; // CorrelationGrid

#endif
